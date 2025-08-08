#include "spac2_0/fmath.h"
#include "spac2_0/utils.h"

using namespace std;

int CommonBase::index = 0;

Pure_Pursuit::Pure_Pursuit(float k_dd, float k_curv, float k_dist, float distance_to_rear_axle)
{
    this->k_dd = k_dd;
    this->k_curv = k_curv;
    this->k_dist = k_dist;
    this->distance_imu_to_rear_axle = -distance_to_rear_axle;
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
    //WARNING: THIS WILL BE NECESSARY FOR ANYOTHER MISSION APART FROM THE SKIDPAD UNTIL THE SLAM IS DONE
    array<float, 2> position = {0.0, 0.0};
    position[0] = static_cast<float>(current_pose.position.x);
    position[1] = static_cast<float>(current_pose.position.y);
    

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

    float look_ahead_distance = 0.0;
    // Calculate look ahead point based on the speed with a min and max distances
    if(this->accel_flag)
    {
        look_ahead_distance = this->k_dist;
    }else{
        look_ahead_distance = clamp(speed_to_lookahead(speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    }
    
    
    // Find the closest point to the look ahead distance intersecting the path with a circle
    optional<array<float, 2>> closest_point = get_closest_point(path_points, look_ahead_distance);
    // if there is no intersection with the path, keep the car straight (?) TODO: check if this is the best approach
    if (!closest_point.has_value())
    {
        return getAvgAngle();
    }
    //if the x value is 0 (straight line) return 0 (no steering angle needed) or else it will give the wrong angle in the atan2
    if ((*closest_point)[0] == 0)
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

    // Calculate steering angle (pure pursuit algorithm)
    float steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), look_ahead_distance);

    // Keep previous angles to calculate the average
    keepAvgAngle(steering_angle);
    
    target_point[0] = (*closest_point)[0];
    target_point[1] = (*closest_point)[1];
    set_target_point(target_point);

    //write the steering angle and the point of intersection to a file
    // ofstream myfile;
    // myfile.open("steer_point.csv", ios::app);
    // myfile << steering_angle * 180 / M_PI << ", " << (*closest_point)[0] << ", " << (*closest_point)[1] << "\n"; 
    // myfile.close();

    return getAvgAngle();
}

float Pure_Pursuit::calculate_desiredSpeed(lart_msgs::msg::PathSpline path, float max_rpm){
    if(index > -1){
        float curvature = abs(path.curvature[index]);
        float p_curv = min(0.95f, curvature * this->k_curv);
        float desired_speed = max_rpm * (1 - p_curv);
        
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "max speed fmath=%f", max_rpm);
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "curvature=%f", curvature);
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "p_curv=%f", p_curv);
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "desired_speed from fmath=%f", desired_speed);

        //RCLCPP_INFO(rclcpp::get_logger("pure"), "Percentagem de velocidade=%f %%", (1 - p_curv) * 100);
        
        return desired_speed;
    }
    return 0.0;
}

void Pure_Pursuit::keepAvgAngle(float steering_angle){
    int slot = cycles % SIZE_AVG_ARRAY;
    avg_angle[slot] = steering_angle;
    cycles++;
}

float Pure_Pursuit::getAvgAngle(){
    float sum = 0;
    int interval = SIZE_AVG_ARRAY;

    //intialize the interval to the number of cycles if it is less than the size of the array
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

array<float, 2> Pure_Pursuit::get_target_point()
{
    return this->target_point;
}

void Pure_Pursuit::set_target_point(array<float, 2> closest_point)
{
    this->target_point = closest_point;
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

void Pure_Pursuit::acc_mode(){
    RCLCPP_INFO(rclcpp::get_logger("fmath"), "Acceleration mode activated");
    this->accel_flag = true;
}

float speed_to_lookahead(float speed){
    float look_ahead_distance = 3.6f + 0.00495614f * speed;
    
    return look_ahead_distance;
}

void Pure_Pursuit::set_ekf(geometry_msgs::msg::Pose pose){
    //this->current_pose = {0.0,0.0};
    (void)pose;
}