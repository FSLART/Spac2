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
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "lart_common.h"

#define PARAMS_DISTANCE_IMU_TO_REAR_AXLE "distance_imu_to_rear_axle"
#define PARAMS_FREQUENCY "frequency"
#define PARAMS_DESIDERED_SPEED "desired_speed"
#define PARAMS_KP_SPEED "kp_speed"
#define PARAMS_KI_SPEED "ki_speed"
#define PARAMS_KD_SPEED "kd_speed"
#define PARAMS_KDD "k_dd"
#define PARAMS_TOPIC_PATH "path_topic"
#define PARAMS_TOPIC_ACKERMANN "ackermann_topic"
#define PARAMS_TOPIC_WHEELS "wheels_topic"

class SpacNode : public rclcpp::Node
{
public:

    SpacNode();

private:
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_path;
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr subscription_wheels;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher;

protected:

    void dispatchAckermannDrive();
    void timer_callback();
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void wheels_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg);

    int lResult = 1;
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
    std::string ackermann_topic;
    std::string wheels_topic;
};

int open_actuation();
int actuate(int actuator_angle);
int close_actuation();


#endif