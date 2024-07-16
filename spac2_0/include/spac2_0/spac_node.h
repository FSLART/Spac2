#ifndef SPAC_NODE_H_
#define SPAC_NODE_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "target.h"
#include "std_msgs/msg/float32.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "../lart_common/lart_common.h"

#define PARAMS_DISTANCE_IMU_TO_REAR_AXLE "distance_imu_to_rear_axle"
#define PARAMS_FREQUENCY "frequency"
#define PARAMS_DESIDERED_SPEED "desired_speed"
#define PARAMS_KP_SPEED "kp_speed"
#define PARAMS_KI_SPEED "ki_speed"
#define PARAMS_KD_SPEED "kd_speed"
#define PARAMS_KDD "k_dd"
#define PARAMS_TOPIC_PATH "path_topic"
#define PARAMS_TOPIC_DYNAMICS_CMD "dynamics_cmd_topic"
#define PARAMS_TOPIC_RPM "rpm_topic"

class SpacNode : public rclcpp::Node
{
public:

    SpacNode();

private:
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_path;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr subscription_rpm;
    rclcpp::Publisher<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_ready;

protected:

    void dispatchDynamicsCMD();
    void timer_callback();
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void rpm_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);

    float distance_imu_to_rear_axle;
    int frequency=0;
    float desired_speed;
    float kp_speed;
    float ki_speed;
    float kd_speed;
    float k_dd_pp;
    int desired_rpm;
    Target *target;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer_publisher;
    std::string path_topic;
    std::string dynamics_cmd_topic;
    std::string rpm_topic;
};

#endif