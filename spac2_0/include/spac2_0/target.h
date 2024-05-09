
#ifndef TARGET_H_
#define TARGET_H_

#include "fmath.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include "utils.h"
#include <rclcpp/logging.hpp>


class Target{
    public:
        Target(int desired_rpm, float kp_speed, float ki_speed, float kd_speed);
        Target(Pure_Pursuit pure_pursuit, PID_Controller pid);
        float get_steering_angle(nav_msgs::msg::Path path, int rpm);
        float get_PID_rpm(float setpoint, float input);
        void instance_CarrotControl();
        ackermann_msgs::msg::AckermannDriveStamped get_dirtyDispatcherMail();
        bool get_isDispatcherDirty();
        int set_throwDirtDispatcher();
        void set_path(nav_msgs::msg::Path path);
        nav_msgs::msg::Path get_path();
        void set_rpm(int rpm);
        int get_rpm();

    protected:
        Pure_Pursuit pure_pursuit;
        PID_Controller pid;
        bool isDispatcherDirty=true;
        nav_msgs::msg::Path path;
        float current_rpm=0;
		ackermann_msgs::msg::AckermannDrive dispatcherMailBox;
        ackermann_msgs::msg::AckermannDriveStamped dispatcherMailBoxStamped;
        int desired_rpm; 
};

#endif