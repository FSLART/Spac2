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
    this->declare_parameter(PARAMS_DESIDERED_SPEED, DEFAULT_DESIRED_SPEED);
    this->get_parameter(PARAMS_DESIDERED_SPEED, desired_speed);
    this->declare_parameter(PARAMS_KP_SPEED, DEFAULT_KP_SPEED);
    this->get_parameter(PARAMS_KP_SPEED, kp_speed);
    this->declare_parameter(PARAMS_KI_SPEED, DEFAULT_KI_SPEED);
    this->get_parameter(PARAMS_KI_SPEED, ki_speed);
    this->declare_parameter(PARAMS_KD_SPEED, DEFAULT_KD_SPEED);
    this->get_parameter(PARAMS_KD_SPEED, kd_speed);
    this->declare_parameter(PARAMS_KDD, DEFAULT_KDD);
    this->get_parameter(PARAMS_KDD, k_dd_pp);
    //topics
    this->declare_parameter(PARAMS_TOPIC_PATH, "/path");
	this->get_parameter(PARAMS_TOPIC_PATH, path_topic);
	this->declare_parameter(PARAMS_TOPIC_DYNAMICS_CMD, "/cmd");
	this->get_parameter(PARAMS_TOPIC_DYNAMICS_CMD, dynamics_cmd_topic);
    this->declare_parameter(PARAMS_TOPIC_RPM, "/rpm");
	this->get_parameter(PARAMS_TOPIC_RPM, rpm_topic);
    
    //convert speed from km/h to m/s
    float speed_mps = desired_speed / 3.6;
    
    //calculate the desired rpm
    desired_rpm = MS_TO_RPM(speed_mps);
    //RCLCPP_INFO(this->get_logger(), "Desired RPM IN NODE: %d", desired_rpm);
    target = new Target(desired_rpm, kp_speed, ki_speed, kd_speed, k_dd_pp);

    //create publisher for ackermann drive
	dynamics_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(dynamics_cmd_topic, 10);

    //receives the current path and calls the path_callback function
    subscription_path = this->create_subscription<nav_msgs::msg::Path>(
        path_topic, 10, std::bind(&SpacNode::path_callback, this, _1));

    subscription_rpm = this->create_subscription<lart_msgs::msg::Dynamics>(
        rpm_topic, 10, std::bind(&SpacNode::rpm_callback, this, _1));

    //TODO: AXANATO PARA AGORA MAS PRECISA DE SER ALTERADO / NO ENTANTO ESTA VALIDAÇÃO É NECESSÁRIA
    subscription_ready = this->create_subscription<std_msgs::msg::Bool>(
        "acu_origin/res_ready", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data)
            {
                RCLCPP_INFO(this->get_logger(), "Received ready signal");
                this->target->set_ready();
            }
        });

    //TODO APAGAR
    //this->target->set_ready();

    auto interval = std::chrono::duration<double>(1.0 / frequency);

    //creates a timer that calls the instance_CarrotControl function
	//RCLCPP_INFO(this->get_logger(), "Started carrot waypoint targeting routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer = this->create_wall_timer(interval, std::bind(&Target::instance_CarrotControl, this->target));

    //creates a timer that calls the dispatchDynamicsCMD function
    //RCLCPP_INFO(this->get_logger(), "Started dynamics command dispatch routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer_publisher= this->create_wall_timer(interval, [this]()-> void {this->dispatchDynamicsCMD();});

}

void SpacNode::dispatchDynamicsCMD(){
	if(this->target->get_isDispatcherDirty()){
		RCLCPP_INFO(this->get_logger(), "Dispatching dynamics cmd on { %s }", __PRETTY_FUNCTION__); 
		this->dynamics_publisher->publish(this->target->get_dirtyDispatcherMail());
		this->target->set_throwDirtDispatcher(); 

	}
}

void SpacNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.c_str());
    this->target->set_path(*msg);
}

void SpacNode::rpm_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    this->target->set_rpm(msg->rpm);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpacNode>());
    rclcpp::shutdown();
    return 0;
}