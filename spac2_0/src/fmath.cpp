#include "spac2_0/fmath.h"
#include "spac2_0/utils.h"

using namespace std;

int CommonBase::index = 0;

Pure_Pursuit::Pure_Pursuit(float k_dd, float k_curv, float k_dist,float distance_to_rear_axle)
{
    this->k_dd = k_dd;
    this->k_curv = k_curv;
    this->k_dist = k_dist;
    this->distance_imu_to_rear_axle = distance_to_rear_axle;
}

//TODO: check how to not have the need for a empty constructor
Pure_Pursuit::Pure_Pursuit()
{
    //this->k_dd = 2.0;
}

float Pure_Pursuit::get_k_dd()
{
    return k_dd;
}


float Pure_Pursuit::calculate_steering_angle(lart_msgs::msg::PathSpline path, float speed)
{
    // Create a pose with the current position  of the car.
    array<float, 2> position = {0.0, 0.0};

    // adds the current position to an array with all the points of the path
    // discarding Z axis
    vector<array<float, 2>> path_points;
    path_points.push_back(position);

    // Define the transformation
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(this->distance_imu_to_rear_axle, 0.0, 0.0));
    transform.setRotation(tf2::Quaternion(0, 0, 0, 1));

    for (long unsigned int i = 0; i < path.poses.size(); i++)
    {
        // Create an array with X and Y position of the path, shifting the X value to the rear of the car
        geometry_msgs::msg::PoseStamped input_pose = path.poses[i];
        tf2::Transform input_transform;
        tf2::fromMsg(input_pose.pose, input_transform);

        // Apply the transformation
        tf2::Transform transformed_transform = transform * input_transform;
        geometry_msgs::msg::Pose transformed_pose;
        transformed_pose.position.x = transformed_transform.getOrigin().x();
        transformed_pose.position.y = transformed_transform.getOrigin().y();
        transformed_pose.position.z = transformed_transform.getOrigin().z();
        transformed_pose.orientation.x = transformed_transform.getRotation().x();
        transformed_pose.orientation.y = transformed_transform.getRotation().y();
        transformed_pose.orientation.z = transformed_transform.getRotation().z();
        transformed_pose.orientation.w = transformed_transform.getRotation().w();

        // Extract the transformed X and Y positions
        std::array<float, 2> point = {
            static_cast<float>(transformed_pose.position.x),
            static_cast<float>(transformed_pose.position.y)
        };

        path_points.push_back(point);
    }
    //RCLCPP_INFO(rclcpp::get_logger("pure"), "k_dd=%f", k_dd);
    // Calculate look ahead point based on the speed with a min and max distances
    //TODO REVERTER

    float look_ahead_distance = clamp(speed_to_lookahead(speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    //RCLCPP_INFO(rclcpp::get_logger("pure"), "lookahead=%f", look_ahead_distance);
    
    //float look_ahead_distance = clamp(k_dd, MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    
    // Find the closest point to the look ahead distance intersecting the path with a circle
    optional<array<float, 2>> closest_point = get_closest_point(path_points, look_ahead_distance);
    array<float, 2> target_point;

    if (!closest_point.has_value())
    {
        return getAvgAngle();
    }
    //if the x value is 0 (straight line) return 0 (no steering angle needed) or else it will give the wrong angle in the atan2
    if ((*closest_point)[1] == 0)
    {
        // Keep previous angles to calculate the average
        keepAvgAngle(0.0f);

        target_point[0] = (*closest_point)[0];
        target_point[1] = (*closest_point)[1];

        set_target_point(target_point);
        return getAvgAngle();
    }

    // Calculate angle between the closest point and (0,0) (because the point is returned relative to (0,0)) instead of the rear!!
    float alpha = atan2((*closest_point)[1], (*closest_point)[0]);
    //RCLCPP_INFO(rclcpp::get_logger("calculate_steering_angle"), "alpha=%f", alpha);
    // Calculate steering angle (pure pursuit algorithm)
    float steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), look_ahead_distance);

    // Keep previous angles to calculate the average
    keepAvgAngle(steering_angle);

    target_point[0] = (*closest_point)[0];
    target_point[1] = (*closest_point)[1];
    set_target_point(target_point);

    //write the steering angle and the point of intersection to a file
    ofstream myfile;
    myfile.open("steer_point.csv", ios::app);
    myfile << steering_angle * 180 / M_PI << ", " << (*closest_point)[0] << ", " << (*closest_point)[1] << "\n"; 
    myfile.close();


    return getAvgAngle();
}

array<float, 2> Pure_Pursuit::get_target_point()
{
    return this->target_point;
}

void Pure_Pursuit::set_target_point(array<float, 2> closest_point)
{
    this->target_point = closest_point;
}


float Pure_Pursuit::calculate_desiredSpeed(lart_msgs::msg::PathSpline path, float max_rpm){
    if(index > -1){
        float curvature = abs(path.curvature[index + this->k_dist]);
        float p_curv = min(1.0f, curvature * this->k_curv);
        float desired_speed = max_rpm * (1 - p_curv);

        //debug
        //RCLCPP_INFO(rclcpp::get_logger("pure"), "curvature=%f", curvature);

        // RCLCPP_INFO(rclcpp::get_logger("pure"), "p_curv=%f", p_curv);
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "max_rpm=%f", max_rpm);

        //RCLCPP_INFO(rclcpp::get_logger("pure"), "Percentagem de velocidade=%f %%", (1 - p_curv) * 100);

        return desired_speed;
    }
    return 0;
}

void Pure_Pursuit::keepAvgAngle(float steering_angle){
    int slot = cycles % SIZE_AVG_ARRAY;
    avg_angle[slot] = steering_angle;
    cycles++;
}

float Pure_Pursuit::getAvgAngle(){
    float sum = 0;
    int interval = SIZE_AVG_ARRAY;
    
    if(cycles < SIZE_AVG_ARRAY){
        if (cycles == 0){
            return 0.0;
        }
        interval = cycles;
    }

    for(int i = 0; i < interval; i++){
        sum += avg_angle[i];
    }
    return sum / interval;
}

optional<array<float, 2>> get_closest_point(vector<array<float, 2>> path_points, float look_ahead_distance)
{
    if(path_points.size() > MIN_INDEX)
    {
        CommonBase::index = fastRound((look_ahead_distance)/AVG_DISTANCE) - 1;
        return path_points[CommonBase::index];
    }
    CommonBase::index = -1;
    return nullopt;
}

int fastRound(float x) {
    return static_cast<int>(x + 0.5f);
}

float speed_to_lookahead(float speed){
    // RCLCPP_INFO(rclcpp::get_logger("pure"), "speed=%f", speed);

    //min lookahead = 7.0
    //float look_ahead_distance = 6.6852f * pow(1.00041, speed);

    //min lookahead = 5.0
    //float look_ahead_distance = 4.732881f * pow(1.000575, speed);

    //recent function
    float look_ahead_distance = (4.62281f + 0.00495614f * speed);

    RCLCPP_INFO(rclcpp::get_logger("pure"), "lookahead=%f", look_ahead_distance);
    return look_ahead_distance;
}

// float Pure_Pursuit::speed_to_kcurv(float speed){
//     float k_curv = 0.00435996 * speed - 1.513;

//     RCLCPP_INFO(rclcpp::get_logger("pure"), "kcurv=%f", k_curv);

//     if(k_curv < 0.0f){
//         k_curv = 0.0f;
//     }
//     return k_curv;
// }