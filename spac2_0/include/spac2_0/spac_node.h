/**
 * @file spac_node.h
 * @author Tomás Santos (2230717@my.ipleiria.pt)
 * @brief A ros node responsible for managing the control of the vehicle
 * @version 0.1
 * 
 */

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
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_common.h"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <ctime>
#include <chrono>

#define PARAMS_DISTANCE_IMU_TO_REAR_AXLE "distance_imu_to_rear_axle"
#define PARAMS_FREQUENCY "frequency"
#define PARAMS_MAX_SPEED "max_speed"
#define PARAMS_ACC_SPEED "acc_speed"
#define PARAMS_EBS_SPEED "ebs_speed"
#define PARAMS_KDD "k_dd"
#define PARAMS_K_CURV "k_curv"
#define PARAMS_K_DIST "k_dist"
#define PARAMS_TOPIC_PATH "path_topic"
#define PARAMS_TOPIC_DYNAMICS_CMD "dynamics_cmd_topic"
#define PARAMS_TOPIC_RPM "rpm_topic"
#define PARAMS_TOPIC_STATE "state_topic"
#define PARAMS_TOPIC_MISSION "mission_topic"
#define PARAMS_GROWTH_FACTOR "growth_factor"
#define PARAMS_LIMITER "max_limit"
#define PARAMS_INCREMENT "increment"
#define PARAMS_ACC_INCREMENT "acc_increment"

#define PARAMS_TARGET_MARKER "target_marker_topic"

class SpacNode : public rclcpp::Node
{
public:

    SpacNode();

private:
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<lart_msgs::msg::PathSpline>::SharedPtr subscription_path;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr subscription_rpm;
    rclcpp::Publisher<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_publisher;
    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_subscriber;

protected:
    //Functions

    /**
    * @brief A function responsible for publishing the speed and steering angle
    * to the state controller.
    * 
    */
    void dispatchDynamicsCMD();
    /**
    * @brief Receives the path from the path planner.
    * 
    * @param msg 
    */
    void path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg);
    /**
    * @brief Receives the rpm from the state controller.
    * 
    * @param msg 
    */
    void rpm_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    /**
    * @brief Receives the state from the state controller.
    */
    void state_callback(const lart_msgs::msg::State::SharedPtr msg);
    /**
    * @brief Receives the mission from the mission_controller.
    */
    void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
    /**
    * @brief A function responsible for sending a clean up message
    * to ensure that random values aren't left forgotten in the system 
    * when the node is shutdown.
    */

    void ekf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    void cleanUp();
    //void whatTimeIsIt();

    //Variables
    float distance_imu_to_rear_axle;    /**< The distance from the IMU to the rear axle in meters */
    int frequency=0;    /**< The frequency of the publisher */
    float max_speed;    /**< The maximum desired speed in km/h*/
    float acc_speed;    /**< The max speed for the aceleration mission */
    float ebs_speed;    /**<The max speed for the EBS test */
    float k_dd_pp;      /**< Lookahead distance if the variable lookahead method isn't being used */
    float k_curv;       /**< Factor of deceleration in function of the curvature of the path */
    float k_dist;       /**< Extra value to be added to the index of the target point, used to get a new target point but only for the longitudinal control */
    float max_rpm;      /**< The maximum desired speed in rpm */
    float growth_factor;
    float increment;   /**< The base limit of the soft start, used to calculate the maximum allowed change of rpm speed between iterations */
    float acc_increment;
    float max_limit; 
    Target *target;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer_publisher;
    std::string path_topic;
    std::string dynamics_cmd_topic;
    std::string rpm_topic;
    std::string state_topic;
    std::string mission_topic;
    std::string target_marker_topic;

    //std::chrono::time_point<std::chrono::system_clock> last_time;
};

#endif