#include "spac2_0/target.h"

Target::Target(int desired_rpm){
    this->pure_pursuit = Pure_Pursuit();
    //TODO: CHANGE TO SET A MAX VALUE THAT IS NOT THE TERMINAL RPM (?)
    this->pid = PID_Controller(0, TERMINAL_RPM);
    //TODO: THERE IS THE NEED TO SET THE PARAMETERS FOR THE PID CONTROLLER: IT COMES FROM THE PARAMETERS -> TO IMPLEMENT
    this->desired_rpm = desired_rpm;
}

Target::Target(Pure_Pursuit pure_pursuit, PID_Controller pid){
    this->pure_pursuit = pure_pursuit;
    this->pid = pid;
}

void Target::instance_CarrotControl(){
    try{
        //current speed is being obtained from the rpm
        //and it is used to calculate how far is the look ahead point
        auto steering_angle = this->get_steering_angle(this->path, 417);
        //clamp steering angle to -MAX_STEERING and MAX_STEERING
        steering_angle = std::clamp(steering_angle, (float)-MAX_STEERING,(float) MAX_STEERING);

        auto rpm = this->get_PID_rpm(current_rpm, desired_rpm);
        //clamp speed to -MAX_SPEED and MAX_SPEED
        rpm = std::clamp(rpm, (float)-TERMINAL_RPM,(float) TERMINAL_RPM);

        //TODO: SETTING ROS MESSAGE WITH RPM AND STEERING ANGLE
        //for now rpms obtained from the PID are being converted to m/s to send with the ackermann message
        auto speed = rpm_to_mps(rpm);

        dispatcherMailBox = ackermann_msgs::msg::AckermannDrive();
        dispatcherMailBox.speed = speed;
        dispatcherMailBox.steering_angle = steering_angle;

        //for now lets keep it simple, this makes it as fast as possible
        dispatcherMailBox.steering_angle_velocity = 0.0f;

        isDispatcherDirty = true;
    }catch(...){

		// Makes shure the dispatcher wont look for bad data
        isDispatcherDirty = false;
    }
}

ackermann_msgs::msg::AckermannDrive Target::get_dirtyDispatcherMail(){
	//This may look "optimizable" but the reason its like this is to keep a error by default approach 
	if(isDispatcherDirty){
		return dispatcherMailBox;
	}
	//Log warning that the dispatcher tried to read clean data
	RCLCPP_WARN(rclcpp::get_logger("get_dirtyDispatcherMail"), "Dispatcher is trying to read clean data, this means that the dispatcher is trying to read data that has not been updated yet");
	return dispatcherMailBox;
} 

bool Target::get_isDispatcherDirty(){
	return isDispatcherDirty;
}

int Target::set_throwDirtDispatcher(){
	if(!isDispatcherDirty){
		RCLCPP_WARN(rclcpp::get_logger("set_throwDirtDispatcher"), "Dispatcher is dirty, this means that the dispatcher is trying to send bad data");
		return -1;
	}
	isDispatcherDirty = false;
	return 0;
}

void Target::set_path(nav_msgs::msg::Path path){
    this->path = path;
}

nav_msgs::msg::Path Target::get_path(){
    return this->path;
}

void Target::set_rpm(int rpm){
    this->current_rpm = rpm;
}

int Target::get_rpm(){
    return this->current_rpm;
}


float Target::get_steering_angle(nav_msgs::msg::Path path, int rpm){
    auto speed = rpm_to_mps(rpm);
    float steering_angle = this->pure_pursuit.calculate_steering_angle(path, speed);
    return steering_angle;
}

float Target::get_PID_rpm(float desired, float current){
    float rpm = this->pid.compute(desired, current);
    return rpm;
}


