#include <gtest/gtest.h>
#include "lart_msgs/msg/path_spline.hpp"
#include "spac2_0/target.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#define TWOCUTFLOATING(x) ((int)(x*100.0f))

TEST (tst_target, get_straight_angle){
    lart_msgs::msg::PathSpline path;
    geometry_msgs::msg::PoseStamped pose_stamped;
    //current_rpm influences the lookahed distance that influences the steering angle
    int current_rpm = 400;
    float expected_steering_angle = 0.0;
    geometry_msgs::msg::Point point;

    std::vector<std::vector<float>> data = {
    {0.0, 0.0, 0, 0},
    {0.5, 0.5, 0, 0},
    {1.0, 1.0, 0, 0},
    {1.5, 1.5, 0, 0},
    {2.0, 2.0, 0, 0},
    {2.5, 2.5, 0, 0},
    {3.0, 3.0, 0, 0},
    {3.5, 3.5, 0, 0},
    {4.0, 4.0, 0, 0},
    {4.5, 4.5, 0, 0},
    {5.0, 5.0, 0, 0},
    {5.5, 5.5, 0, 0}
    };

    // Populate the Pathspline message
    for (const auto& row : data) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Add distance and curvature
        path.distance.push_back(distance);
        path.curvature.push_back(curvature);

        // Create and populate a PoseStamped
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)

        // Add the PoseStamped to the pathspline
        path.poses.push_back(pose_stamped);
    }

    Target target(0.0, 0.1, 0.1, 0.1, 0, 0, 5.2, 1.15);
    float steering_angle = target.get_steering_angle(path, current_rpm);
    std::cerr << "[          ] steering angle = " << steering_angle << std::endl;
    ASSERT_NEAR((int)TWOCUTFLOATING(steering_angle), (int) TWOCUTFLOATING(expected_steering_angle),1);
}

TEST (tst_target, not_enough_points){
    lart_msgs::msg::PathSpline path;
    geometry_msgs::msg::PoseStamped pose_stamped;
    //current_rpm influences the lookahed distance that influences the steering angle
    int current_rpm = 400;
    float expected_steering_angle = 0.0;
    geometry_msgs::msg::Point point;

    std::vector<std::vector<float>> data = {
    {0.0, 0.0, 0, 0},
    {0.5, 0.5, 0, 0}
    };

    // Populate the Pathspline message
    for (const auto& row : data) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Add distance and curvature
        path.distance.push_back(distance);
        path.curvature.push_back(curvature);

        // Create and populate a PoseStamped
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)

        // Add the PoseStamped to the pathspline
        path.poses.push_back(pose_stamped);
    }

    Target target(0.0, 0.1, 0.1, 0.1, 0, 0, 5.2, 1.15);
    float steering_angle = target.get_steering_angle(path, current_rpm);
    std::cerr << "[          ] steering angle = " << steering_angle << std::endl;
    ASSERT_NEAR((int)TWOCUTFLOATING(steering_angle), (int) TWOCUTFLOATING(expected_steering_angle),1);
}

// TEST (tst_target, get_angle_invalid_path){
//     lart_msgs::msg::PathSpline path;
//     //current_rpm influences the lookahed distance that influences the steering angle
//     int current_rpm = 400;      //current speed: 2.67 m/s (9,61 km/h)
//     float expected_steering_angle = 0.0;
//     geometry_msgs::msg::Point point;
//     geometry_msgs::msg::Quaternion quaternion;
//     point.x = -5.0;
//     point.y = -5.0;
//     point.z = 0.0;
//     quaternion.x = 0.0;
//     quaternion.y = 0.0;
//     quaternion.z = 0.0;
//     quaternion.w = 1.0;
//     geometry_msgs::msg::Pose pose;
//     pose.position = point;
//     pose.orientation = quaternion;
//     geometry_msgs::msg::PoseStamped pose_stamped;
//     pose_stamped.pose = pose;
//     path.poses.push_back(pose_stamped);
//     Target target(417, 0.1, 0.1, 0.1, 2.0);
//     float steering_angle = target.get_steering_angle(path, current_rpm);
//     //std::cerr << "[          ] steering angle = " << steering_angle << std::endl;
//     ASSERT_NEAR((int)steering_angle, (int) expected_steering_angle, 1);
// }

// TEST (tst_target, get_angle_invalid_path_negative_x){
//     lart_msgs::msg::PathSpline path;
//     //current_rpm influences the lookahed distance that influences the steering angle
//     int current_rpm = 400;      //current speed: 2.67 m/s (9,61 km/h)
//     float expected_steering_angle = 0.0;
//     geometry_msgs::msg::Point point;
//     geometry_msgs::msg::Quaternion quaternion;
//     point.x = -5.0;
//     point.y = 5.0;
//     point.z = 0.0;
//     quaternion.x = 0.0;
//     quaternion.y = 0.0;
//     quaternion.z = 0.0;
//     quaternion.w = 1.0;
//     geometry_msgs::msg::Pose pose;
//     pose.position = point;
//     pose.orientation = quaternion;
//     geometry_msgs::msg::PoseStamped pose_stamped;
//     pose_stamped.pose = pose;
//     path.poses.push_back(pose_stamped);
//     Target target(417, 0.1, 0.1, 0.1, 2.0);
//     float steering_angle = target.get_steering_angle(path, current_rpm);
//     //std::cerr << "[          ] steering angle = " << steering_angle << std::endl;
//     ASSERT_NEAR((int)steering_angle, (int) expected_steering_angle, 1);
// }

TEST (tst_target, get_angle_example_path){
    lart_msgs::msg::PathSpline path;
    geometry_msgs::msg::PoseStamped pose_stamped;
    //current_rpm influences the lookahed distance that influences the steering angle
    int current_rpm = 400;      //current speed: 2.67 m/s (9,61 km/h)
    float expected_steering_angle = 0.1636456282582117;

    std::vector<std::vector<float>> data = {
    {0.00000000, -0.0578711390, -0.0419723335, 0.102172062},
    {0.495972149, 0.434555014, 0.00524532866, 0.101734447},
    {0.991944299, 0.924145585, 0.0778424859, 0.101140748},
    {1.48791645, 1.40977053, 0.175242234, 0.100426292},
    {1.98388860, 1.89029981, 0.296867669, 0.0996595820},
    {2.47986075, 2.36460338, 0.442141886, 0.0988950161},
    {2.97583290, 2.83155120, 0.610487981, 0.0981850733},
    {3.47180505, 3.29001322, 0.801329050, 0.0976202298},
    {3.96777719, 3.73885941, 1.01408819, 0.0972945139},
    {4.46374934, 4.17695972, 1.24818849, 0.0971543551},
    {4.95972149, 4.60318411, 1.50305306, 0.0971408345},
    {5.45569364, 5.01642145, 1.77808304, 0.0971882216},
    {5.95166579, 5.41567802, 2.07254340, 0.0971428136},
    {6.61296199, 5.92463255, 2.49402155, 0.0963946603},
    {7.10893414, 6.28757809, 2.83075777, 0.0947358350},
    {7.60490629, 6.63338275, 3.18429920, 0.0916536840},
    {8.10087844, 6.96168975, 3.55363493, 0.0870351641},
    {8.59685059, 7.27354658, 3.93722361, 0.0812775739},
    {9.09282274, 7.57020522, 4.33344663, 0.0752264248},
    {9.58879489, 7.85291764, 4.74068540, 0.0698884581},
    {10.0847670, 8.12292003, 5.15733122, 0.0660754073},
    {10.5807392, 8.38067892, 5.58225773, 0.0641273599},
    {11.0767113, 8.62556037, 6.01502828, 0.0638730678},
    {11.5726835, 8.85684583, 6.45525921, 0.0648440188},
    {12.0686556, 9.07381674, 6.90256689, 0.0665448745},
    {12.5646278, 9.27575456, 7.35656768, 0.0686021969},
    {13.2259240, 9.52037822, 7.97165050, 0.0716213266},
    {13.7218961, 9.68444444, 8.43977661, 0.0741421681},
    {14.2178683, 9.83108238, 8.91331704, 0.0770414185},
    {14.7138404, 9.95957350, 9.39188813, 0.0805075865},
    {15.2098126, 10.0691822, 9.87509668, 0.0846988386},
    {15.7057847, 10.1587391, 10.3623063, 0.0896639917},
    {16.2017569, 10.2266172, 10.8526239, 0.0953073757},
    {16.6977290, 10.2711677, 11.3451441, 0.101411374},
    {17.1937012, 10.2907415, 11.8389616, 0.107618648},
    {17.6896733, 10.2836898, 12.3331711, 0.113146443},
    {18.1856455, 10.2483636, 12.8268671, 0.117800039},
    {18.6816176, 10.1831140, 13.3191443, 0.121537275},
    {19.1775898, 10.0862921, 13.8090973, 0.124380425},
    {19.8388860, 9.90523370, 14.4571879, 0.126969180}
    };

    // Populate the Pathspline message
    for (const auto& row : data) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Add distance and curvature
        path.distance.push_back(distance);
        path.curvature.push_back(curvature);

        // Create and populate a PoseStamped
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)

        // Add the PoseStamped to the pathspline
        path.poses.push_back(pose_stamped);
    }

    Target target(0.0, 0.1, 0.1, 0.1, 1.5, 10.0, 5.2, 1.15);
    float steering_angle = target.get_steering_angle(path, current_rpm);
    std::cerr << "[          ] steering angle = " << steering_angle << std::endl;
    ASSERT_NEAR((int) TWOCUTFLOATING(steering_angle), (int) TWOCUTFLOATING(expected_steering_angle), 1);
 }

TEST (tst_target, get_angle_three_cycles){
    lart_msgs::msg::PathSpline path;
    lart_msgs::msg::PathSpline path2;
    lart_msgs::msg::PathSpline path3;
    geometry_msgs::msg::PoseStamped pose_stamped;
    //current_rpm influences the lookahed distance that influences the steering angle
    int current_rpm = 400;      //current speed: 2.67 m/s (9,61 km/h)
    float expected_steering_angle = 0.25194386;

    std::vector<std::vector<float>> data = {
    {0.00000000, -0.0578711390, -0.0419723335, 0.102172062},
    {0.495972149, 0.434555014, 0.00524532866, 0.101734447},
    {0.991944299, 0.924145585, 0.0778424859, 0.101140748},
    {1.48791645, 1.40977053, 0.175242234, 0.100426292},
    {1.98388860, 1.89029981, 0.296867669, 0.0996595820},
    {2.47986075, 2.36460338, 0.442141886, 0.0988950161},
    {2.97583290, 2.83155120, 0.610487981, 0.0981850733},
    {3.47180505, 3.29001322, 0.801329050, 0.0976202298},
    {3.96777719, 3.73885941, 1.01408819, 0.0972945139},
    {4.46374934, 4.17695972, 1.24818849, 0.0971543551},
    {4.95972149, 4.60318411, 1.50305306, 0.0971408345},
    {5.45569364, 5.01642145, 1.77808304, 0.0971882216}, // -> 0.1636456282582117
    {5.95166579, 5.41567802, 2.07254340, 0.0971428136},
    {6.61296199, 5.92463255, 2.49402155, 0.0963946603},
    {7.10893414, 6.28757809, 2.83075777, 0.0947358350}
    };

    // Populate the Pathspline message
    for (const auto& row : data) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Add distance and curvature
        path.distance.push_back(distance);
        path.curvature.push_back(curvature);

        // Create and populate a PoseStamped
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)

        // Add the PoseStamped to the pathspline
        path.poses.push_back(pose_stamped);
    }
    Target target(0.0, 0.1, 0.1, 0.1, 1.5, 1.0, 5.2, 1.15);
    float steering_angle = target.get_steering_angle(path, current_rpm);
    std::cerr << "[          ] steering angle = " << steering_angle << std::endl;

    std::vector<std::vector<float>> data2 = {
    {0.00000000, -0.0578711390, -0.0419723335, 0.102172062},
    {0.495972149, 0.434555014, 0.00524532866, 0.101734447},
    {0.991944299, 0.924145585, 0.0778424859, 0.101140748},
    {1.48791645, 1.40977053, 0.175242234, 0.100426292},
    {1.98388860, 1.89029981, 0.296867669, 0.0996595820},
    {2.47986075, 2.36460338, 0.442141886, 0.0988950161},
    {2.97583290, 2.83155120, 0.610487981, 0.0981850733},
    {3.47180505, 3.29001322, 0.801329050, 0.0976202298},
    {3.96777719, 3.73885941, 1.01408819, 0.0972945139},
    {4.46374934, 4.17695972, 1.24818849, 0.0971543551},
    {4.95972149, 4.60318411, 1.50305306, 0.0971408345},
    {5.45569364, 4.77000000, 2.64000000, 0.0971882216}, // -> 0.2379761052640517
    {5.95166579, 5.41567802, 2.07254340, 0.0971428136},
    {6.61296199, 5.92463255, 2.49402155, 0.0963946603},
    {7.10893414, 6.28757809, 2.83075777, 0.0947358350}
    };

    for (const auto& row : data2) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Add distance and curvature
        path2.distance.push_back(distance);
        path2.curvature.push_back(curvature);

        // Create and populate a PoseStamped
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)

        // Add the PoseStamped to the pathspline
        path2.poses.push_back(pose_stamped);
    }

    steering_angle = target.get_steering_angle(path2, current_rpm);
    std::cerr << "[          ] steering angle = " << steering_angle << std::endl;

    std::vector<std::vector<float>> data3 = {
    {0.00000000, -0.0578711390, -0.0419723335, 0.102172062},
    {0.495972149, 0.434555014, 0.00524532866, 0.101734447},
    {0.991944299, 0.924145585, 0.0778424859, 0.101140748},
    {1.48791645, 1.40977053, 0.175242234, 0.100426292},
    {1.98388860, 1.89029981, 0.296867669, 0.0996595820},
    {2.47986075, 2.36460338, 0.442141886, 0.0988950161},
    {2.97583290, 2.83155120, 0.610487981, 0.0981850733},
    {3.47180505, 3.29001322, 0.801329050, 0.0976202298},
    {3.96777719, 3.73885941, 1.01408819, 0.0972945139},
    {4.46374934, 4.17695972, 1.24818849, 0.0971543551},
    {4.95972149, 4.60318411, 1.50305306, 0.0971408345},
    {5.45569364, 5.19000000, 5.01000000, 0.0971882216}, // -> 0.35421294
    {5.95166579, 5.41567802, 2.07254340, 0.0971428136},
    {6.61296199, 5.92463255, 2.49402155, 0.0963946603},
    {7.10893414, 6.28757809, 2.83075777, 0.0947358350}
    };

    for (const auto& row : data3) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Add distance and curvature
        path3.distance.push_back(distance);
        path3.curvature.push_back(curvature);

        // Create and populate a PoseStamped
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)

        // Add the PoseStamped to the pathspline
        path3.poses.push_back(pose_stamped);
    }

    steering_angle = target.get_steering_angle(path3, current_rpm);
    std::cerr << "[          ] steering angle = " << steering_angle << std::endl;

    ASSERT_NEAR((int) TWOCUTFLOATING(steering_angle), (int) TWOCUTFLOATING(expected_steering_angle), 1);
 }
