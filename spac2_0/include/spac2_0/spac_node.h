#ifndef SPAC_NODE_H_
#define SPAC_NODE_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "target.h"
#include "std_msgs/msg/float32.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "lart_common.h"
#include "lart_msgs/msg/path_spline.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"

#include <ctime>

#define PARAMS_DISTANCE_IMU_TO_REAR_AXLE "distance_imu_to_rear_axle"
#define PARAMS_FREQUENCY "frequency"
#define PARAMS_MAX_SPEED "max_speed"
#define PARAMS_KP_SPEED "kp_speed"
#define PARAMS_KI_SPEED "ki_speed"
#define PARAMS_KD_SPEED "kd_speed"
#define PARAMS_KDD "k_dd"
#define PARAMS_K_CURV "k_curv"
#define PARAMS_K_DIST "k_dist"
#define PARAMS_TOPIC_PATH "path_topic"
#define PARAMS_TOPIC_WHEELS "wheels_topic"
#define PARAMS_TOPIC_ACKERMANN "ackermann_topic"
#define PARAMS_TOPIC_STATE "state_topic"
#define PARAMS_TOPIC_MISSION "mission_topic"

#define PARAMS_TARGET_MARKER "target_marker_topic"

class SpacNode : public rclcpp::Node
{
public:

    SpacNode();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<lart_msgs::msg::PathSpline>::SharedPtr subscription_path;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher;
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr subscription_wheels;



protected:

    void dispatchAckermannDrive();
    void timer_callback();
    void wheels_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg);
    void path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg);
    void cleanUp();
    //void whatTimeIsIt();

    float distance_imu_to_rear_axle;
    int frequency=0;
    float max_speed;
    float k_dd_pp;
    float k_curv;
    float k_dist;
    float max_rpm;
    Target *target;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer_publisher;
    std::string path_topic;
    std::string wheels_topic;
    std::string ackermann_topic;
    std::string state_topic;
    std::string mission_topic;
    std::string target_marker_topic;

    //std::chrono::time_point<std::chrono::system_clock> last_time;
};

#endif