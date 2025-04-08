#include "spac2_0/spac_node.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

SpacNode::SpacNode() : Node("spac_node")
{
    this->declare_parameter(PARAMS_FREQUENCY, DEFAULT_FREQUENCY);
	this->get_parameter(PARAMS_FREQUENCY, frequency);
    //TODO: check if it makes sense to have the distance coming from the parameters, if so needs to be passed to the pure_pursuit object, for now using the default there
    this->declare_parameter(PARAMS_DISTANCE_IMU_TO_REAR_AXLE, DEFAULT_IMU_TO_REAR_AXLE);
	this->get_parameter(PARAMS_DISTANCE_IMU_TO_REAR_AXLE, distance_imu_to_rear_axle);
    this->declare_parameter(PARAMS_MAX_SPEED, DEFAULT_MAX_SPEED);
    this->get_parameter(PARAMS_MAX_SPEED, max_speed);

    this->declare_parameter(PARAMS_KDD, DEFAULT_KDD);
    this->get_parameter(PARAMS_KDD, k_dd_pp);
    this->declare_parameter(PARAMS_K_CURV, DEFAULT_K_CURV);
    this->get_parameter(PARAMS_K_CURV, k_curv);
    this->declare_parameter(PARAMS_K_DIST, DEFAULT_K_DIST);
    this->get_parameter(PARAMS_K_DIST, k_dist);
    //topics
    this->declare_parameter(PARAMS_TOPIC_PATH, "/path");
	this->get_parameter(PARAMS_TOPIC_PATH, path_topic);
    this->declare_parameter(PARAMS_TOPIC_ACKERMANN, "/cmd");
	this->get_parameter(PARAMS_TOPIC_ACKERMANN, ackermann_topic);
    
    this->declare_parameter(PARAMS_TOPIC_WHEELS, "/wheel_speeds");
	this->get_parameter(PARAMS_TOPIC_WHEELS, wheels_topic);
    this->declare_parameter(PARAMS_TOPIC_STATE, "/pc_origin/system_status/critical_as/state");
    this->get_parameter(PARAMS_TOPIC_STATE, state_topic);
    this->declare_parameter(PARAMS_TOPIC_MISSION, "/mission");
    this->get_parameter(PARAMS_TOPIC_MISSION, mission_topic);

    this->declare_parameter(PARAMS_TARGET_MARKER, "/target_marker_topic");
    this->get_parameter(PARAMS_TARGET_MARKER, target_marker_topic);
    
    // RCLCPP_INFO(this->get_logger(), "porp: %f", kp_speed);
    // RCLCPP_INFO(this->get_logger(), "inte: %f", ki_speed);
    // RCLCPP_INFO(this->get_logger(), "deriv: %f", kd_speed);


    // Debug statements to verify parameters
    // RCLCPP_INFO(this->get_logger(), "Path topic: %s", path_topic.c_str());
    // RCLCPP_INFO(this->get_logger(), "Dynamics CMD topic: %s", ackermann_topic.c_str());
    // RCLCPP_INFO(this->get_logger(), "Wheels topic: %s", wheels_topic.c_str());
    // RCLCPP_INFO(this->get_logger(), "Target marker topic: %s", target_marker_topic.c_str());


    //convert speed from km/h to m/s
    float speed_mps = max_speed / 3.6;
    RCLCPP_INFO(this->get_logger(), "Defined max speed: %f", speed_mps);

    //calculate the desired rpm
    max_rpm = MS_TO_RPM(speed_mps);

    //RCLCPP_INFO(this->get_logger(), "MAX SPEED: %f", max_speed);
    RCLCPP_INFO(this->get_logger(), "Defined max rpm: %f", max_rpm);

    

    //RCLCPP_INFO(this->get_logger(), "Desired RPM IN NODE: %d", desired_rpm);
    target = new Target(max_rpm, k_curv, k_dist, k_dd_pp, distance_imu_to_rear_axle);

    // Create a publisher for visualization markers
    marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(target_marker_topic, 10);

    //create publisher for ackermann drive
	ackermann_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);

    //receives the current path and calls the path_callback function
    subscription_path = this->create_subscription<lart_msgs::msg::PathSpline>(
        path_topic, 10, std::bind(&SpacNode::path_callback, this, _1));

    subscription_wheels = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
        wheels_topic, 10, std::bind(&SpacNode::wheels_callback, this, _1));

    //PARA TESTES
    this->target->set_ready();

    auto interval = std::chrono::duration<double>(1.0 / frequency);

    //creates a timer that calls the instance_CarrotControl function
	//RCLCPP_INFO(this->get_logger(), "Started carrot waypoint targeting routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer = this->create_wall_timer(interval, std::bind(&Target::instance_CarrotControl, this->target));

    //creates a timer that calls the dispatchDynamicsCMD function
    //RCLCPP_INFO(this->get_logger(), "Started dynamics command dispatch routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer_publisher= this->create_wall_timer(interval, [this]()-> void {this->dispatchAckermannDrive();});


    rclcpp::on_shutdown([this]() {
        cleanUp();
    });


}

void SpacNode::dispatchAckermannDrive(){
	if(this->target->get_isDispatcherDirty()){
		//RCLCPP_INFO(this->get_logger(), "Dispatching dynamics cmd on { %s }", __PRETTY_FUNCTION__); 

        //debug
        ackermann_msgs::msg::AckermannDriveStamped dispatcherMailBoxStamped = ackermann_msgs::msg::AckermannDriveStamped();
        dispatcherMailBoxStamped = this->target->get_dirtyDispatcherMail();
        ackermann_msgs::msg::AckermannDrive dispatcherMailBox = dispatcherMailBoxStamped.drive;

        //RCLCPP_INFO(this->get_logger(), "Speed: %f", dispatcherMailBox.speed);
        //RCLCPP_INFO(this->get_logger(), "Steering: %f", dispatcherMailBox.steering_angle);


		this->ackermann_publisher->publish(this->target->get_dirtyDispatcherMail());
        visualization_msgs::msg::Marker marker = this->target->get_target_marker();

        this->marker_publisher->publish(marker);

		this->target->set_throwDirtDispatcher(); 

	}
}

void SpacNode::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "NEW Path received");
    // for(long unsigned int i=0; i < msg->poses.size(); i++){
    //     RCLCPP_INFO(this->get_logger(), "I heard (X): '%f'", msg->poses[i].pose.position.x);
    //     RCLCPP_INFO(this->get_logger(), "I heard (Y): '%f'", msg->poses[i].pose.position.y);
    // }
    this->target->set_path(*msg);

    //whatTimeIsIt();
}

void SpacNode::wheels_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->speeds.lb_speed);
    float speed = ((msg->speeds.lb_speed + msg->speeds.rb_speed)/2) / 37.8188;
    this->target->set_rpm(MS_TO_RPM(speed));
}



void SpacNode::cleanUp()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up");
    ackermann_msgs::msg::AckermannDrive cleanUpMailBox = ackermann_msgs::msg::AckermannDrive();
    cleanUpMailBox.speed = 0.0;
    cleanUpMailBox.steering_angle = 0.0;

    ackermann_msgs::msg::AckermannDriveStamped cleanUpMailBoxStamped;
    cleanUpMailBoxStamped.drive = cleanUpMailBox;

    this->ackermann_publisher->publish(cleanUpMailBoxStamped);
}

// void SpacNode::whatTimeIsIt(){
//     std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_time);
//     //RCLCPP_INFO(this->get_logger(), "Time since last path: %ld ms", duration.count());

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