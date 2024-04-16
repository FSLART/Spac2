#ifndef SPAC_NODE_H_
#define SPAC_NODE_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "target.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/float32.hpp"

#define PARAMS_DISTANCE_IMU_TO_REAR_AXLE "distance_imu_to_rear_axle"
#define PARAMS_FREQUENCY "frequency"
#define PARAMS_DESIDERED_SPEED "desired_speed"
#define PARAMS_KP_SPEED "kp_speed"
#define PARAMS_KI_SPEED "ki_speed"
#define PARAMS_KD_SPEED "kd_speed"
#define PARAMS_TOPIC_PATH "path_topic"
#define PARAMS_TOPIC_ACKERMANN "ackermann_topic"
#define PARAMS_TOPIC_RPM "rpm_topic"

class SpacNode : public rclcpp::Node
{
public:

    SpacNode();

private:
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_path;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_rpm;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_publisher;

protected:

    void dispatchAckermannDrive();
    void timer_callback();
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);

    float distance_imu_to_rear_axle;
    int frequency=0;
    float desired_speed;
    float kp_speed;
    float ki_speed;
    float kd_speed;
    int desired_rpm;
    Target *target;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer_publisher;
    std::string path_topic;
    std::string ackermann_topic;
    std::string rpm_topic;
};

#endif