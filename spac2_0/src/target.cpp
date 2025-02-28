#include "spac2_0/target.h"

Target::Target(float max_rpm, float kp_speed, float ki_speed, float kd_speed, float k_curv, float k_dist, float kdd, float distance_imu_to_rear_axle){
    this->pure_pursuit = Pure_Pursuit(kdd, k_curv, k_dist, distance_imu_to_rear_axle);
    //TODO: CHANGE TO SET A MAX VALUE THAT IS NOT THE TERMINAL RPM (?)
    this->pid = PID_Controller(0, TERMINAL_RPM);
    this->pid.set_Tunings(kp_speed, ki_speed, kd_speed);
    this->max_rpm = clamp(max_rpm,(float)0.0,(float)TERMINAL_RPM);
    RCLCPP_WARN(rclcpp::get_logger("instance_CarrotControl"), "Constructor_rpm=%f",this->max_rpm);
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
        ////RCLCPP(rclcpp::get_logger("instance_CarrotControl"), "k_dd=%f", this->pure_pursuit.get_k_dd());
        //current speed is being obtained from the rpm
        //and it is used to calculate how far is the look ahead point
        auto steering_angle = this->get_steering_angle(this->path, this->current_rpm);
        //clamp steering angle to -MAX_STEERING and MAX_STEERING
        steering_angle = std::clamp((float)(steering_angle), (float)-MAX_WHEEL_ANGLE_RAD,(float) MAX_WHEEL_ANGLE_RAD);


        //CREATING THE TARGET POINT MARKER
        array<float, 2> target_point = this->pure_pursuit.get_target_point();
        this->set_target_marker(target_point);

        //NOTE: COMENTED OUT BECAUSE IT IS NOT BEING USED IN THE TESTS
        float desired_rpm = this->get_desired_rpm(this->path, this->max_rpm);

        auto rpm = this->get_PID_rpm(desired_rpm, this->current_rpm);
        //clamp speed to -MAX_SPEED and MAX_SPEED
        //TODO: -TERMINAL_RPM DOES NOT MAKE THAT MUCH SENSE
        rpm = std::clamp(rpm, (float)0.0,(float)this->max_rpm);

        //RCLCPP_WARN(rclcpp::get_logger("instance_CarrotControl"), "rpm=%f", rpm);

        RCLCPP_INFO(rclcpp::get_logger("instance_CarrotControl"), "DESIRED_rpm=%f", desired_rpm);
        RCLCPP_INFO(rclcpp::get_logger("instance_CarrotControl"), "pid_rpm=%f", rpm);

        auto speed = RPM_TO_MS(rpm);

        //create dispatcher with rpm and steering
        dispatcherMailBox = ackermann_msgs::msg::AckermannDrive();
        dispatcherMailBox.speed = speed;
        dispatcherMailBox.steering_angle = steering_angle;

        //RCLCPP(rclcpp::get_logger("instance_CarrotControl"), "steering=%f", dispatcherMailBox.steering_angle);

        isDispatcherDirty = true;
    }catch(...){

		// Makes shure the dispatcher wont look for bad data
        isDispatcherDirty = false;
    }
}


ackermann_msgs::msg::AckermannDriveStamped Target::get_dirtyDispatcherMail(){
	//This may look "optimizable" but the reason its like this is to keep a error by default approach
    dispatcherMailBoxStamped.drive = dispatcherMailBox;

	if(isDispatcherDirty){

        // RCLCPP_WARN(rclcpp::get_logger("get_dirtyDispatcherMail"), " " );
        // RCLCPP_WARN(rclcpp::get_logger("get_dirtyDispatcherMail"), "dispatcherMailBox=%f", dispatcherMailBox.steering_angle);
		return dispatcherMailBoxStamped;
	}
    //TODO: WHEN THERE IS A CATCH IN THE INSTANCE_CARROTCONTROL FUNCTION, this will say that it tried to read clean but it can be bad data!!
	//Log warning that the dispatcher tried to read clean data
	RCLCPP_WARN(rclcpp::get_logger("get_dirtyDispatcherMail"), "Dispatcher is trying to read clean data, this means that the dispatcher is trying to read data that has not been updated yet");
    return dispatcherMailBoxStamped;
} 

void Target::set_target_marker(array<float, 2> target_point){

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "pure_pursuit";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = target_point[0];
    marker.pose.position.y = target_point[1];
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = rclcpp::Duration::from_seconds(1);
    
    this->target_marker = marker;
}

visualization_msgs::msg::Marker Target::get_target_marker(){
    return this->target_marker;
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

void Target::set_path(lart_msgs::msg::PathSpline path){
    this->path = path;
}

lart_msgs::msg::PathSpline Target::get_path(){
    return this->path;
}

void Target::set_rpm(int rpm){
    this->current_rpm = rpm;
}

int Target::get_rpm(){
    return this->current_rpm;
}


float Target::get_steering_angle(lart_msgs::msg::PathSpline path, int rpm){
    float steering_angle = this->pure_pursuit.calculate_steering_angle(path, rpm);
    return steering_angle;
}

float Target::get_PID_rpm(float desired, float current){
    float rpm = this->pid.compute(desired, current);
    return rpm;
}

float Target::get_desired_rpm(lart_msgs::msg::PathSpline path, float max_rpm){
    float desired_rpm = this->pure_pursuit.calculate_desiredSpeed(path, max_rpm);
    //float desired_rpm = MS_TO_RPM(speed);
    //RCLCPP_INFO(rclcpp::get_logger("get_desired_rpm"), "desiredRpm=%f", desired_rpm);
    return desired_rpm;
}


