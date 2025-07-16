#include "spac2_0/spac_node.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std::chrono;

SpacNode::SpacNode() : Node("spac_node")
{
    this->declare_parameter(PARAMS_FREQUENCY, DEFAULT_FREQUENCY);
	this->get_parameter(PARAMS_FREQUENCY, frequency);
    this->declare_parameter(PARAMS_DISTANCE_IMU_TO_REAR_AXLE, DEFAULT_IMU_TO_REAR_AXLE);
    this->get_parameter(PARAMS_DISTANCE_IMU_TO_REAR_AXLE, distance_imu_to_rear_axle);
    
    //MAX SPEED
    this->declare_parameter(PARAMS_MAX_SPEED, DEFAULT_MAX_SPEED);
    this->get_parameter(PARAMS_MAX_SPEED, max_speed);
    
    //Pure Pursuit parameters
    this->declare_parameter(PARAMS_KDD, DEFAULT_KDD);
    this->get_parameter(PARAMS_KDD, k_dd_pp);
    this->declare_parameter(PARAMS_K_CURV, DEFAULT_K_CURV);
    this->get_parameter(PARAMS_K_CURV, k_curv);
    this->declare_parameter(PARAMS_K_DIST, DEFAULT_K_DIST);
    this->get_parameter(PARAMS_K_DIST, k_dist);
    
    //topics
    this->declare_parameter(PARAMS_TOPIC_PATH, "/path");
	this->get_parameter(PARAMS_TOPIC_PATH, path_topic);
	this->declare_parameter(PARAMS_TOPIC_DYNAMICS_CMD, "/cmd");
	this->get_parameter(PARAMS_TOPIC_DYNAMICS_CMD, dynamics_cmd_topic);
    this->declare_parameter(PARAMS_TOPIC_RPM, "/rpm");
	this->get_parameter(PARAMS_TOPIC_RPM, rpm_topic);
    this->declare_parameter(PARAMS_TOPIC_STATE, "/pc_origin/system_status/critical_as/state");
    this->get_parameter(PARAMS_TOPIC_STATE, state_topic);
    this->declare_parameter(PARAMS_TOPIC_MISSION, "/mission");
    this->get_parameter(PARAMS_TOPIC_MISSION, mission_topic);

    // soft start variables
    this->declare_parameter(PARAMS_GROWTH_FACTOR, DEFAULT_GROWTH_FACTOR);
    this->get_parameter(PARAMS_GROWTH_FACTOR, growth_factor);
    this->declare_parameter(PARAMS_BASE_LIMIT, DEFAULT_BASE_LIMIT);
    this->get_parameter(PARAMS_BASE_LIMIT, base_limit);
    this->declare_parameter(PARAMS_LIMITER, DEFAULT_LIMITER);
    this->get_parameter(PARAMS_LIMITER, max_limit);

    //Visualization
    this->declare_parameter(PARAMS_TARGET_MARKER, "/target_marker_topic");
    this->get_parameter(PARAMS_TARGET_MARKER, target_marker_topic);
    
    //convert speed from km/h to m/s
    float speed_mps = max_speed / 3.6;
    
    //calculate the desired rpm
    max_rpm = MS_TO_RPM(speed_mps);

    //DEBUG
    //RCLCPP_INFO(this->get_logger(), "Defined max rpm: %f", max_rpm);
    
    //RCLCPP_INFO(this->get_logger(), "Desired RPM IN NODE: %d", desired_rpm);
    target = new Target(max_rpm, k_curv, k_dist, k_dd_pp, distance_imu_to_rear_axle, growth_factor, base_limit, max_limit);

    // Create a publisher for visualization markers
    marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(target_marker_topic, 10);
    path_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/spac/path_marker", 10);

    //create publisher for ackermann drive
	dynamics_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(dynamics_cmd_topic, 10);

    //receives the current path and calls the path_callback function
    subscription_path = this->create_subscription<lart_msgs::msg::PathSpline>(
        path_topic, 10, std::bind(&SpacNode::path_callback, this, _1));
    
    //receives the current rpm and calls the rpm_callback function
    subscription_rpm = this->create_subscription<lart_msgs::msg::Dynamics>(
        rpm_topic, 10, std::bind(&SpacNode::rpm_callback, this, _1));

    //receives the current state and calls the state_callback function
    state_subscriber = this->create_subscription<lart_msgs::msg::State>(
        state_topic, 10, std::bind(&SpacNode::state_callback, this, _1));

    mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(
        mission_topic, 10, std::bind(&SpacNode::mission_callback, this, _1));

    //receives ekf imu data
    ekf_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ekf/state", 10, std::bind(&SpacNode::ekf_callback, this, _1));

    // APENAS USAR NOS TESTES
    this->target->set_ready();

    auto interval = std::chrono::duration<double>(1.0 / frequency);

    //creates a timer that calls the instance_CarrotControl function
	//RCLCPP_INFO(this->get_logger(), "Started carrot waypoint targeting routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer = this->create_wall_timer(interval, std::bind(&Target::instance_CarrotControl, this->target));

    //creates a timer that calls the dispatchDynamicsCMD function
    //RCLCPP_INFO(this->get_logger(), "Started dynamics command dispatch routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer_publisher= this->create_wall_timer(interval, [this]()-> void {this->dispatchDynamicsCMD();});

    rclcpp::on_shutdown([this]() {
        cleanUp();
    });
}

void SpacNode::ekf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "EKF callback received");
    this->target->set_ekf(msg->pose);
}

void SpacNode::state_callback(const lart_msgs::msg::State::SharedPtr msg){
    if(msg->data == lart_msgs::msg::State::DRIVING){
        RCLCPP_INFO(this->get_logger(), "Received DRIVING signal");
        this->target->set_ready();
    }
    if(msg->data == lart_msgs::msg::State::FINISH){
        RCLCPP_INFO(this->get_logger(), "Received FINISH signal");
        this->cleanUp();
        this->target->disengage_ready();
    }
    if(msg->data == lart_msgs::msg::State::EMERGENCY){
        RCLCPP_INFO(this->get_logger(), "Received EMERGENCY signal");
        this->cleanUp();
        this->target->disengage_ready();
    }
}

void SpacNode::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg){
    if(msg->data == lart_msgs::msg::Mission::ACCELERATION){
        RCLCPP_INFO(this->get_logger(), "Received ACCELERATION MISSION");
        this->target->set_acceleration_mission();
    }
}

void SpacNode::dispatchDynamicsCMD(){
	if(this->target->get_isDispatcherDirty()){
		//RCLCPP_INFO(this->get_logger(), "Dispatching dynamics cmd on { %s }", __PRETTY_FUNCTION__);
		

        this->dynamics_publisher->publish(this->target->get_dirtyDispatcherMail());

        //Sending marker
        visualization_msgs::msg::Marker marker = this->target->get_target_marker();
        this->marker_publisher->publish(marker);
        //Sending path marker
        visualization_msgs::msg::Marker path_marker = this->target->get_path_marker();
        this->path_marker_publisher->publish(path_marker);

		this->target->set_throwDirtDispatcher(); 

	}
}

void SpacNode::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.c_str());
    this->target->set_path(*msg);
    //whatTimeIsIt();
}

void SpacNode::rpm_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    this->target->set_rpm(msg->rpm);
}

void SpacNode::cleanUp()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up");
    lart_msgs::msg::DynamicsCMD cleanUpMailBox = lart_msgs::msg::DynamicsCMD();
    cleanUpMailBox.rpm = 0;
    cleanUpMailBox.steering_angle = 0.0;

    this->dynamics_publisher->publish(cleanUpMailBox);
}

// void SpacNode::whatTimeIsIt(){
//     auto now = std::chrono::system_clock::now();

//     auto duration = duration_cast<milliseconds>(now - this->last_time);
//     // RCLCPP_INFO(this->get_logger(), "Time since last path: %ld ms", duration.count());

//     this->last_time = now;

//     ofstream myfile;
//     myfile.open("testing_path_time.csv", ios::app);
//     myfile << duration.count() << "\n"; 
//     myfile.close();

// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpacNode>());
    rclcpp::shutdown();
    return 0;
}