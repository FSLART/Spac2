
#ifndef TARGET_H_
#define TARGET_H_

#include "fmath.h"
#include "lart_common.h"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include <cmath>
#include "utils.h"
#include <rclcpp/logging.hpp>
#include "visualization_msgs/msg/marker.hpp"


class Target{
    public:
        // Functions

        /**
        * @brief Constructor for the Target class.
        * 
        * @param max_rpm
        * @param k_curv 
        * @param k_dist 
        * @param kdd 
        * @param distance_imu_to_rear_axle
        * 
        */
        Target(float max_rpm, float k_curv, float k_dist, float kdd, float distance_imu_to_rear_axle);
        /**
        * @brief Constructor for the Target class.
        *
        * @param pure_pursuit
        * 
        */
        Target(Pure_Pursuit pure_pursuit);
        /**
        * @brief Calls the the pure_pursuit.calculate_steering_angle function to calculate the steering angle.
        *
        * @param path
        * @param rpm
        *
        * @return steering angle
        */
        float get_steering_angle(lart_msgs::msg::PathSpline path, int rpm);
        /**
        * @brief Calls the the pure_pursuit.calculate_desiredSpeed function to calculate the desired speed in rpm.
        * 
        * @param path
        * @param max_rpm
        * 
        * @return desired speed in rpm
        */
        float get_desired_rpm(lart_msgs::msg::PathSpline path, float max_rpm);
        /**
        * @brief Function that is periodically called to get the
        * calculated values of the steering angle and speed needed
        * in a certain point.
        * 
        * These two values are used to fill a dynamicsCMD msg. 
        */
        void instance_CarrotControl();
        /**
        * @brief This fuction is used to return the values that were
        * obtained in the instace_CarrotControl.
        * 
        * @return DispatcherMailBox
        */

        void set_acceleration_mission();
        lart_msgs::msg::DynamicsCMD get_dirtyDispatcherMail();
        bool get_isDispatcherDirty();
        int set_throwDirtDispatcher();
        void set_path(lart_msgs::msg::PathSpline path);
        lart_msgs::msg::PathSpline get_path();
        void set_rpm(int rpm);
        int get_rpm();
        void set_ready();
        void disengage_ready();
        bool get_ready();
        void set_target_marker(array<float, 2> target_point);
        visualization_msgs::msg::Marker get_target_marker();

    protected:
        //Variables
        Pure_Pursuit pure_pursuit;                      /**< Object of the class Pure_Pursuit */
        bool isDispatcherDirty=true;                    /**< Flag used in the get_dirtyDispatcherMail function */
        lart_msgs::msg::PathSpline path;                /**< The path obtained from the path planner at a certain moment */
        float current_rpm=0;                            /**< The rpm of the motor in at a certain moment */
        lart_msgs::msg::DynamicsCMD dispatcherMailBox;  /**< This is a DynamicsCMD msg that will store the speed and rpm */
        float max_rpm;
        float k_curv;
        float k_dist;
        bool ready=false;
        bool acel_flag=false;

        visualization_msgs::msg::Marker target_marker;
};

#endif