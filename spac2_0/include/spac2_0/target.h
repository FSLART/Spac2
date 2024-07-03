
#ifndef TARGET_H_
#define TARGET_H_

#include "fmath.h"
#include "../lart_common/lart_common.h" 
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include <cmath>
#include "utils.h"
#include <rclcpp/logging.hpp>


class Target{
    public:
        Target(int desired_rpm, float kp_speed, float ki_speed, float kd_speed, float kdd);
        Target(Pure_Pursuit pure_pursuit, PID_Controller pid);
        float get_steering_angle(nav_msgs::msg::Path path, int rpm);
        float get_PID_rpm(float setpoint, float input);
        void instance_CarrotControl();
        lart_msgs::msg::DynamicsCMD get_dirtyDispatcherMail();
        bool get_isDispatcherDirty();
        int set_throwDirtDispatcher();
        void set_path(nav_msgs::msg::Path path);
        nav_msgs::msg::Path get_path();
        void set_rpm(int rpm);
        int get_rpm();
        void set_ready();
        bool get_ready();

    protected:
        Pure_Pursuit pure_pursuit;
        PID_Controller pid;
        bool isDispatcherDirty=true;
        nav_msgs::msg::Path path;
        float current_rpm=0;
		lart_msgs::msg::DynamicsCMD dispatcherMailBox;
        int desired_rpm; 
        bool ready=false;
};

#endif