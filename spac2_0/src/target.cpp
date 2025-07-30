#include "spac2_0/target.h"

Target::Target(float max_rpm, float k_curv, float k_dist, float kdd, float distance_imu_to_rear_axle, float increment){
    this->pure_pursuit = Pure_Pursuit(kdd, k_curv, k_dist, distance_imu_to_rear_axle);
    this->max_rpm = std::clamp(max_rpm, (float)0.0, (float)TERMINAL_RPM);
    this->increment = increment;
}

Target::Target(Pure_Pursuit pure_pursuit){
    this->pure_pursuit = pure_pursuit;
}

void Target::instance_CarrotControl(){
    try{
        if(get_ready() == false){
            throw std::runtime_error("Not ready yet");
        }
        
        auto steering_angle = this->get_steering_angle(this->path, this->current_rpm);
        
        //clamp steering angle to -MAX_STEERING and MAX_STEERING
        steering_angle = std::clamp((float)(steering_angle), (float)-MAX_WHEEL_ANGLE_RAD,(float) MAX_WHEEL_ANGLE_RAD);


        //CREATING THE TARGET POINT MARKER
        array<float, 2> target_point = this->pure_pursuit.get_target_point();
        this->set_target_marker(target_point);

        //gets the the ideal rpm that the car should have in a certain point of the path
        float rpm = this->get_desired_rpm(this->path, this->max_rpm);
        rpm = std::clamp(rpm, (float)0.0, (float)TERMINAL_RPM);



        /*ORIGINAL CODE*/
        if (rpm - this->last_rpm > this->increment) {
            // RCLCPP_INFO(rclcpp::get_logger("instance_CarrotControl"), "USED LIMIT");
            rpm = this->last_rpm + this->increment;
        }


        RCLCPP_INFO(rclcpp::get_logger("instance_CarrotControl"), "rpm=%f", rpm);

        this->last_rpm = rpm;

        //create dispatcher with rpm and steering
        dispatcherMailBox = lart_msgs::msg::DynamicsCMD();
        dispatcherMailBox.rpm = (int)rpm;

        // if(!this->acel_flag){
        //     //if the car is not in acceleration mission, set the steering angle to the calculated one
        //     dispatcherMailBox.steering_angle = steering_angle;
        // }else{
        //     //in case that the mission is acceleration
        //     dispatcherMailBox.steering_angle = steering_angle/4;
        // }

        dispatcherMailBox.steering_angle = steering_angle;

        //RCLCPP_INFO(rclcpp::get_logger("instance_CarrotControl"), "steering=%f", dispatcherMailBox.steering_angle);
        // RCLCPP_INFO(rclcpp::get_logger("instance_CarrotControl after"), "rpm=%f", rpm);

        //write the steering angle and speed to a file
        // ofstream myfile;
        // myfile.open("dynamics_logger.csv", ios::app);
        // myfile << steering_angle * 180 / M_PI << ", " << rpm << "\n"; 
        // myfile.close();

        isDispatcherDirty = true;
    }catch(...){

		// Makes sure the dispatcher wont look for bad data
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

void Target::set_acceleration_mission(){
    acel_flag = true;
}

void Target::set_ready(){
    ready = true;
}

void Target::disengage_ready(){
    ready = false;
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
    this->current_rpm = (float)rpm;
    RCLCPP_INFO(rclcpp::get_logger("set_rpm"), "rpm=%f", this->current_rpm);
}

int Target::get_rpm(){
    return this->current_rpm;
}

float Target::get_steering_angle(lart_msgs::msg::PathSpline path, int rpm){
    float steering_angle = this->pure_pursuit.calculate_steering_angle(path, rpm);
    return steering_angle;
}


float Target::get_desired_rpm(lart_msgs::msg::PathSpline path, float max_rpm){
    float desired_rpm = this->pure_pursuit.calculate_desiredSpeed(path, max_rpm);
    return desired_rpm;
}

void Target::set_ekf(geometry_msgs::msg::Pose pose){
    this->pure_pursuit.set_ekf(pose);
}

