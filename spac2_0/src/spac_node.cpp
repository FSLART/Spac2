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
    this->declare_parameter(PARAMS_TOPIC_PATH, "/");
	this->get_parameter(PARAMS_TOPIC_PATH, path_topic);
	this->declare_parameter(PARAMS_TOPIC_ACKERMANN, "/");
	this->get_parameter(PARAMS_TOPIC_ACKERMANN, ackermann_topic);
    this->declare_parameter(PARAMS_TOPIC_WHEELS, "/");
	this->get_parameter(PARAMS_TOPIC_WHEELS, wheels_topic);

    //convert speed from km/h to m/s
    float speed_mps = desired_speed / 3.6;
    
    //calculate the desired rpm
    desired_rpm = mps_to_rpm(speed_mps);
    target = new Target(desired_rpm, kp_speed, ki_speed, kd_speed, k_dd_pp);

    //create publisher for ackermann drive
	ackermann_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_topic, 10);

    //receives the current path and calls the path_callback function
    subscription_path = this->create_subscription<nav_msgs::msg::Path>(
        path_topic, 10, std::bind(&SpacNode::path_callback, this, _1));

    subscription_wheels = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
        wheels_topic, 10, std::bind(&SpacNode::wheels_callback, this, _1));

    auto interval = std::chrono::duration<double>(1.0 / frequency);

    //creates a timer that calls the instance_CarrotControl function
	RCLCPP_INFO(this->get_logger(), "Started carrot waypoint targeting routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer = this->create_wall_timer(interval, std::bind(&Target::instance_CarrotControl, this->target));

    //creates a timer that calls the dispatchAckermannDrive function
    RCLCPP_INFO(this->get_logger(), "Started ackermann drive dispatch routine on { %s }", __PRETTY_FUNCTION__ );
	this->timer_publisher= this->create_wall_timer(interval, [this]()-> void {this->dispatchAckermannDrive();});

}

void SpacNode::dispatchAckermannDrive(){
	if(this->target->get_isDispatcherDirty()){
		//RCLCPP_INFO(this->get_logger(), "Dispatching ackermann drive on { %s }", __PRETTY_FUNCTION__); 
		this->ackermann_publisher->publish(this->target->get_dirtyDispatcherMail());
		this->target->set_throwDirtDispatcher(); 

	}
}

void SpacNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "NEW MESSAGE");
    for(long unsigned int i=0; i < msg->poses.size(); i++){
        RCLCPP_INFO(this->get_logger(), "I heard (X): '%f'", msg->poses[i].pose.position.x);
        RCLCPP_INFO(this->get_logger(), "I heard (Y): '%f'", msg->poses[i].pose.position.y);
    }
    this->target->set_path(*msg);
}

void SpacNode::wheels_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->speeds.lb_speed);
    float speed = ((msg->speeds.lb_speed + msg->speeds.rb_speed)/2) / 37.8188;
    this->target->set_rpm(mps_to_rpm(speed));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpacNode>());
    rclcpp::shutdown();
    return 0;
}