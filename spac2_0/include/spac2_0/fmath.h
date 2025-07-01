
#ifndef FMATH_H_
#define FMATH_H_

#include "lart_msgs/msg/path_spline.hpp"
#include <optional>
#include "lart_common.h"
#include <cmath>
#include <rclcpp/logging.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "utils.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

class CommonBase {
public:
    static int index;
};

class Pure_Pursuit : public CommonBase{
    public:
        //Functions
        
        /**
        * @brief Constructor for the Pure_Pursuit class.
        *
        * @param k_dd
        * @param k_curv
        * @param k_dist
        * @param distance_imu_to_rear_axle
        * 
        */
        Pure_Pursuit(float k_dd, float k_curv, float k_dist, float distance_imu_to_rear_axle);
        /**
        * @brief Empty constructor for the Pure_Pursuit class.
        */
        Pure_Pursuit();
        /**
        * @brief Fuction that calculates the steering angle using a target point (PurePursuit).
        * It will return the average of the last 3 calculated steering angle, this is done to 
        * achieve a smoother change between steering angles and to attenuate possible errors
        * caused by wrongly calculated paths.
        *  
        * @param path
        * @param speed
        * 
        * @return steering angle
        */
        float calculate_steering_angle(lart_msgs::msg::PathSpline path, float speed);
        /**
        * @brief Function that calculates the the desired speed of the car in a certain point of the path,
        * using the curvature that is associated with said point. 
        * The value will be returned in rpm. 
        * 
        * @param path
        * @param max_rpm
        * 
        * @return rpm
        */
        float calculate_desiredSpeed(lart_msgs::msg::PathSpline path, float max_rpm);
        /**
        * @brief Function responsible for saving the last 3 calculated steering angles.
        * 
        * @param steering_angle
        */
        void keepAvgAngle(float steering_angle);
        /**
        * @brief Function returns the average of the last 3 calculated steering angles.
        *
        * @return average steering angle.
        */

        /**
         * @brief Function that calculates a lookahead distance acording to a certain speed in rpm
         * 
         * @param speed
         * 
         * @return lookahead
         */
        float speed_to_lookahead(float speed);
        
        float getAvgAngle();
        float get_k_dd();

        array<float, 2> get_target_point();
        void set_target_point(array<float, 2> closest_point);
        void set_ekf(geometry_msgs::msg::Pose pose);
    protected:
        float k_dd;
        float k_curv, k_dist;
        float distance_imu_to_rear_axle;
        float avg_angle[SIZE_AVG_ARRAY] = {0};  /**< Array used to store the last 3 steering angles*/
        int cycles = 0;                         /**< The number of the current iteration */
        array<float, 2> target_point;
        geometry_msgs::msg::Pose current_pose; /**< The current pose of the car */
};

/**
* @brief This function will get a target point acording to the lookahead distance.
* 
* @param path_points
* @param look_ahead_distance
*
* @return target point
*/
optional<array<float, 2>> get_closest_point(vector<array<float, 2>> path_points, float look_ahead_distance);

/**
* @brief Optimized round function, to only take into acount positive values
*/
int fastRound(float x);

#endif