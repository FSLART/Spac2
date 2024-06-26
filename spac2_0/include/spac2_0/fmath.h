
#ifndef FMATH_H_
#define FMATH_H_

#include "nav_msgs/msg/path.hpp"
#include <optional>
#include "../lart_common/lart_common.h"
#include <cmath>
#include <rclcpp/logging.hpp>
#include <iostream>
#include <fstream>

using namespace std;

class Pure_Pursuit{
    public:
        Pure_Pursuit(float k_dd);
        Pure_Pursuit();
        float get_k_dd();
        float calculate_steering_angle(nav_msgs::msg::Path path, float speed);
    protected:
        float k_dd;

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

optional<array<float, 2>> get_closest_point(std::vector<std::array<float,2>> path_points, float look_ahead_distance);
optional<vector<array<float, 2>>> get_intersection(std::array<float, 2> point1, std::array<float, 2> point2, float radius);

#endif