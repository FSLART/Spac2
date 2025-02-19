
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
        Pure_Pursuit(float k_dd, float k_curv, float k_dist, float distance_imu_to_rear_axle);
        Pure_Pursuit();
        float get_k_dd();
        float calculate_steering_angle(lart_msgs::msg::PathSpline path, float speed);
        float calculate_desiredSpeed(lart_msgs::msg::PathSpline path);
        void keepAvgAngle(float steering_angle);
        float getAvgAngle();
        array<float, 2> get_target_point();
        void set_target_point(array<float, 2> closest_point);
    protected:
        float k_dd;
        float k_curv, k_dist;
        float distance_imu_to_rear_axle;
        float avg_angle[SIZE_AVG_ARRAY] = {0};
        int cycles = 0;
        array<float, 2> target_point;
};

class PID_Controller{
    public:
        PID_Controller();
        PID_Controller(float min, float max);
        float compute(float setpoint, float input);
        int set_Tunings(float kp, float ki, float kd);
        float get_Proportion();
        float get_Integral();
        float get_Derivative();
            
    protected:
        float kp, ki, kd, min_signal_value, max_signal_value;
        float output_past, error, error_prev, error_sum;
};

optional<array<float, 2>> get_closest_point(vector<array<float, 2>> path_points, float look_ahead_distance);
int fastRound(float x);
float speed_to_lookahead(float speed);

#endif