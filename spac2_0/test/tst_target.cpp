#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include "spac2_0/target.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#define TWOCUTFLOATING(x) ((int)(x*100.0f))

TEST (tst_target, get_straight_angle){
    nav_msgs::msg::Path path;
    //current_rpm influences the lookahed distance that influences the steering angle
    int cureent_rpm = 400;
    float expected_steering_angle = 0.0;
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Quaternion quaternion;
    point.x = 0.0;
    point.y = 50.0;
    point.z = 0.0;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    quaternion.w = 1.0;
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    path.poses.push_back(pose_stamped);
    Target target(417);
    float steering_angle = target.get_steering_angle(path, cureent_rpm);
    ASSERT_EQ(steering_angle, expected_steering_angle);
}

TEST (tst_target, get_calculated_angle){
    nav_msgs::msg::Path path;
    //current_rpm influences the lookahed distance that influences the steering angle
    int cureent_rpm = 400;      //current speed: 2.67 m/s (9,61 km/h)
    float expected_steering_angle = 39.654786357;
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Quaternion quaternion;
    point.x = 50.0;
    point.y = 50.0;
    point.z = 0.0;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    quaternion.w = 1.0;
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    path.poses.push_back(pose_stamped);
    Target target(417);
    float steering_angle = target.get_steering_angle(path, cureent_rpm);
    ASSERT_EQ(TWOCUTFLOATING(steering_angle), TWOCUTFLOATING(expected_steering_angle));
}