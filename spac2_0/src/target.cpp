#include "spac2_0/target.h"

Target::Target(int desired_rpm, float kp_speed, float ki_speed, float kd_speed, float kdd){
    this->pure_pursuit = Pure_Pursuit(kdd);
    //TODO: CHANGE TO SET A MAX VALUE THAT IS NOT THE TERMINAL RPM (?)
    this->pid = PID_Controller(0, TERMINAL_RPM);
    this->pid.set_Tunings(kp_speed, ki_speed, kd_speed);
    this->desired_rpm = desired_rpm;
}

Target::Target(Pure_Pursuit pure_pursuit, PID_Controller pid){
    this->pure_pursuit = pure_pursuit;
    this->pid = pid;
}

void Target::instance_CarrotControl(){
    try{
        if(get_ready() == false){
            throw std::runtime_error("Not ready yet");
        }
        //print the value of the k_dd of target->pure_pursuit
        RCLCPP_INFO(rclcpp::get_logger("instance_CarrotControl"), "k_dd=%f", this->pure_pursuit.get_k_dd());
        //current speed is being obtained from the rpm
        //and it is used to calculate how far is the look ahead point
        auto steering_angle = this->get_steering_angle(this->path, this->current_rpm);
        //clamp steering angle to -MAX_STEERING and MAX_STEERING
        steering_angle = std::clamp(steering_angle, (float)-SW_ANGLE_TO_ST_ANGLE(MAX_STEERING_ANGLE_RAD),(float) SW_ANGLE_TO_ST_ANGLE(MAX_STEERING_ANGLE_RAD));

        auto rpm = this->get_PID_rpm(current_rpm, desired_rpm);
        //clamp speed to -MAX_SPEED and MAX_SPEED
        //TODO: -TERMINAL_RPM DOES NOT MAKE THAT MUCH SENSE
        rpm = std::clamp(rpm, (float)-TERMINAL_RPM,(float) TERMINAL_RPM);

        //create dispatcher with rpm and steering
        dispatcherMailBox = lart_msgs::msg::DynamicsCMD();
        dispatcherMailBox.rpm = rpm;
        dispatcherMailBox.steering_angle = steering_angle;

        isDispatcherDirty = true;
    }catch(...){

		// Makes shure the dispatcher wont look for bad data
        isDispatcherDirty = false;
    }
}

lart_msgs::msg::DynamicsCMD Target::get_dirtyDispatcherMail(){
	//This may look "optimizable" but the reason its like this is to keep a error by default approach 
	if(isDispatcherDirty){
		return dispatcherMailBox;
	}
    //TODO: WHEN THERE IS A CATCH IN THE INSTANCE_CARROTCONTROL FUNCTION, this will say that it tried to read clean but it can be bad data!!
	//Log warning that the dispatcher tried to read clean data
	RCLCPP_WARN(rclcpp::get_logger("get_dirtyDispatcherMail"), "Dispatcher is trying to read clean data, this means that the dispatcher is trying to read data that has not been updated yet");
	return dispatcherMailBox;
} 

void Target::set_ready(){
    ready = true;
}

bool Target::get_ready(){
    return ready;
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
    auto speed = RPM_TO_MS(rpm);
    float steering_angle = this->pure_pursuit.calculate_steering_angle(path, speed);
    return steering_angle;
}

float Target::get_PID_rpm(float desired, float current){
    float rpm = this->pid.compute(desired, current);
    return rpm;
}


