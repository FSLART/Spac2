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

TEST (tst_target, get_angle_invalid_path){
    nav_msgs::msg::Path path;
    //current_rpm influences the lookahed distance that influences the steering angle
    int cureent_rpm = 400;      //current speed: 2.67 m/s (9,61 km/h)
    float expected_steering_angle = 0.0;
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Quaternion quaternion;
    point.x = -50.0;
    point.y = -50.0;
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
    //std::cerr << "[          ] steering angle = " << steering_angle << std::endl;
    ASSERT_EQ(TWOCUTFLOATING(steering_angle), TWOCUTFLOATING(expected_steering_angle));
}

TEST (tst_target, get_angle_3_points){
    nav_msgs::msg::Path path;
    //current_rpm influences the lookahed distance that influences the steering angle
    int cureent_rpm = 400;      //current speed: 2.67 m/s (9,61 km/h)
    float expected_steering_angle = 44.820065635;   //TODO: the output from the program is 44.24...  Problably just because of some rounding error
    geometry_msgs::msg::Point pointM;
    geometry_msgs::msg::Point pointN;
    geometry_msgs::msg::Point pointO;
    
    geometry_msgs::msg::Quaternion quaternion;
    pointM.x = 0.4;
    pointM.y = 0.45;
    pointM.z = 0.0; //irrelevant
    
    pointN.x = 0.7;
    pointN.y = 1.0;
    pointN.z = 0.0; //irrelevant
    
    pointO.x = 3;
    pointO.y = 2;
    pointO.z = 0.0; //irrelevant
    
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    quaternion.w = 1.0;
    geometry_msgs::msg::Pose poseM;
    poseM.position = pointM;
    poseM.orientation = quaternion;
    geometry_msgs::msg::PoseStamped pose_stampedM;
    pose_stampedM.pose = poseM;
    path.poses.push_back(pose_stampedM);

    geometry_msgs::msg::Pose poseN;
    poseN.position = pointN;
    poseN.orientation = quaternion; //quaternion is irrelevant so it is the same var as the previous one
    geometry_msgs::msg::PoseStamped pose_stampedN;
    pose_stampedN.pose = poseN;
    path.poses.push_back(pose_stampedN);

    geometry_msgs::msg::Pose poseO;
    poseO.position = pointO;
    poseO.orientation = quaternion;
    geometry_msgs::msg::PoseStamped pose_stampedO;
    pose_stampedO.pose = poseO;
    path.poses.push_back(pose_stampedO);

    Target target(417);
    float steering_angle = target.get_steering_angle(path, cureent_rpm);
    //std::cerr << "[          ] steering angle = " << steering_angle << std::endl;
    ASSERT_EQ(TWOCUTFLOATING(steering_angle), TWOCUTFLOATING(expected_steering_angle));
}