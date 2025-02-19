
#ifndef TARGET_H_
#define TARGET_H_


#include "fmath.h"
#include "lart_common.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include "utils.h"
#include <rclcpp/logging.hpp>
#include "visualization_msgs/msg/marker.hpp"


class Target{
    public:
        Target(float kp_speed, float ki_speed, float kd_speed,float k_curv, float k_dist, float kdd, float distance_imu_to_rear_axle);
        Target(Pure_Pursuit pure_pursuit, PID_Controller pid);
        float get_steering_angle(lart_msgs::msg::PathSpline path, int rpm);
        float get_desired_rpm(lart_msgs::msg::PathSpline path);
        float get_PID_rpm(float setpoint, float input);
        void instance_CarrotControl();
        ackermann_msgs::msg::AckermannDriveStamped get_dirtyDispatcherMail();
        bool get_isDispatcherDirty();
        int set_throwDirtDispatcher();
        void set_path(lart_msgs::msg::PathSpline path);
        lart_msgs::msg::PathSpline get_path();
        void set_rpm(int rpm);
        int get_rpm();
        void set_ready();
        bool get_ready();
        void set_target_marker(array<float, 2> target_point);
        visualization_msgs::msg::Marker get_target_marker();

    protected:
        visualization_msgs::msg::Marker target_marker;
        Pure_Pursuit pure_pursuit;
        PID_Controller pid;
        bool isDispatcherDirty=true;
        lart_msgs::msg::PathSpline path;
        float current_rpm=0;
        ackermann_msgs::msg::AckermannDrive dispatcherMailBox;
        ackermann_msgs::msg::AckermannDriveStamped dispatcherMailBoxStamped;
        //int desired_rpm;
        float k_curv;
        float k_dist; 
        bool ready=false;
};

#endif