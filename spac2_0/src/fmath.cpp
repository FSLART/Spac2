#include "spac2_0/fmath.h"
#include "spac2_0/utils.h"

using namespace std;

int CommonBase::index = 0;

Pure_Pursuit::Pure_Pursuit(float k_dd, float k_curv, float k_dist,float distance_to_rear_axle)
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
    // array<float, 2> position = {0.0, 0.0};
    // position[0] = static_cast<float>(current_pose.position.x);
    // position[1] = static_cast<float>(current_pose.position.y);
    

    // adds the current position to an array with all the points of the path
    vector<array<float, 2>> path_points;

    float first_x=0.0;
    float first_y=0.0;

    if(path.poses.size() > MIN_INDEX){
        first_x = path.poses[0].pose.position.x;
        first_y = path.poses[0].pose.position.y;
    }

    // Define the transformation
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(this->distance_imu_to_rear_axle, 0.0, 0.0));

    //transform.setRotation(tf2::Quaternion(0, 0, 0, 1));
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, -this->current_pose.orientation.w);
    transform.setRotation(rotation);


    // DEBUG: CREATE MARKERS FOR THE PATH
    // Create the marker
    visualization_msgs::msg::Marker path_viz_marker;
    path_viz_marker.header.frame_id = "base_footprint";
    path_viz_marker.header.stamp = rclcpp::Clock().now();
    path_viz_marker.ns = "path";
    path_viz_marker.id = 0;
    path_viz_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_viz_marker.action = visualization_msgs::msg::Marker::ADD;



    // Set marker properties
    path_viz_marker.scale.x = 0.1; // Line width

    // Line color (RGBA)
    path_viz_marker.color.r = 1.0;
    path_viz_marker.color.g = 0.0;
    path_viz_marker.color.b = 0.0;
    path_viz_marker.color.a = 1.0;

    
    // RCLCPP_INFO(rclcpp::get_logger("pure"),"Pose: x=%f y=%f",this->current_pose.position.x,this->current_pose.position.y);
    // RCLCPP_INFO(rclcpp::get_logger("pure"),"Heading = %f",this->current_pose.orientation.w);
    for (long unsigned int i = 0; i < path.poses.size(); i++)
    {
        //RCLCPP_INFO(rclcpp::get_logger("pure"),"Before transformation (%f,%f)",path.poses[i].pose.position.x, path.poses[i].pose.position.y);

        // tf to negate the postion of the car in the map
        path.poses[i].pose.position.x -= first_x;
        path.poses[i].pose.position.y -= first_y;

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
        
        // DEBUG: ADD POINTS TO THE PATH MARKER
        geometry_msgs::msg::Point p;
        p.x = static_cast<float>(transformed_pose.position.x);
        p.y = static_cast<float>(transformed_pose.position.y);
        path_viz_marker.points.push_back(p);

        path_points.push_back(point);
        
        //RCLCPP_INFO(rclcpp::get_logger("pure"),"After transformation (%f,%f)",static_cast<float>(transformed_pose.position.x),static_cast<float>(transformed_pose.position.y));
    }

        // path.poses[i].pose.position.x -= first_x;
        // path.poses[i].pose.position.y -= first_y;

        // geometry_msgs::msg::PoseStamped input_pose = path.poses[i];
        // tf2::Transform input_transform;
        // tf2::fromMsg(input_pose.pose, input_transform);

        // // Shift 1.15 meters behind (along local X-axis)
        // tf2::Vector3 local_offset(-1.15, 0.0, 0.0);  // Move backward
        // input_transform.setOrigin(input_transform.getOrigin() + local_offset);

        // // Apply rotation transformation
        // tf2::Transform transformed_transform = transform * input_transform;

        // geometry_msgs::msg::Pose transformed_pose;
        // transformed_pose.position.x = transformed_transform.getOrigin().x();
        // transformed_pose.position.y = transformed_transform.getOrigin().y();
        // transformed_pose.position.z = transformed_transform.getOrigin().z();
        // transformed_pose.orientation.x = transformed_transform.getRotation().x();
        // transformed_pose.orientation.y = transformed_transform.getRotation().y();
        // transformed_pose.orientation.z = transformed_transform.getRotation().z();
        // transformed_pose.orientation.w = transformed_transform.getRotation().w();

        // std::array<float, 2> point = {
        //     static_cast<float>(transformed_pose.position.x),
        //     static_cast<float>(transformed_pose.position.y)
        // };

        // geometry_msgs::msg::Point p;
        // p.x = point[0];
        // p.y = point[1];
        // path_viz_marker.points.push_back(p);

        // path_points.push_back(point);

        // if(i == 0){
        //     RCLCPP_INFO(rclcpp::get_logger("pure"),"x=%f, y=%f",path.poses[i].pose.position.x, path.poses[i].pose.position.y);
        // }

    // DEBBUG: SAVE THE PATH MARKER
    this->path_marker = path_viz_marker;

    // Calculate look ahead point based on the speed with a min and max distances
    float look_ahead_distance = clamp(speed_to_lookahead(speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    
    //float look_ahead_distance = clamp(k_dd, MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    
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
    //float alpha = atan2((*closest_point)[1], (*closest_point)[0]);

    // Calculate angle between the closest point and the first point of the path
    float alpha = atan2((*closest_point)[1], (*closest_point)[0]);

    // Calculate steering angle (pure pursuit algorithm)
    float steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), look_ahead_distance);


    RCLCPP_INFO(rclcpp::get_logger("pure"), "closest point x=%f closest point y=%f", (*closest_point)[0], (*closest_point)[1]);
    // RCLCPP_INFO(rclcpp::get_logger("pure"), "look_ahead_distance=%f", look_ahead_distance);
    // RCLCPP_INFO(rclcpp::get_logger("pure"), "alpha=%f", alpha);
    RCLCPP_INFO(rclcpp::get_logger("pure"), "steering_angle=%f", steering_angle);

    // Keep previous angles to calculate the average
    keepAvgAngle(steering_angle);
    
    target_point[0] = (*closest_point)[0];
    target_point[1] = (*closest_point)[1];
    set_target_point(target_point);

    //write reference point to a file
    // ofstream myfile;
    // myfile.open("spac_analytics.csv", ios::app);
    // myfile << steering_angle * 180 / M_PI << ", " << (*closest_point)[0] << ", " << (*closest_point)[1] << ", " << look_ahead_distance << "\n"; 
    // myfile.close();

    return getAvgAngle();
}

float Pure_Pursuit::calculate_desiredSpeed(lart_msgs::msg::PathSpline path, float max_rpm){
    if(index > -1){
        float curvature = abs(path.curvature[index + this->k_dist]);
        float p_curv = min(0.95f, curvature * this->k_curv);
        float desired_speed = max_rpm * (1 - p_curv);
        
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "max speed fmath=%f", max_rpm);
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "curvature=%f", curvature);
        // RCLCPP_INFO(rclcpp::get_logger("pure"), "p_curv=%f", p_curv);
        RCLCPP_INFO(rclcpp::get_logger("pure"), "desired_speed from fmath=%f", desired_speed);
        
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

float Pure_Pursuit::speed_to_lookahead(float speed){

    //min lookahead = 5.0
    //float look_ahead_distance = 4.732881f * pow(1.000575, speed);

    //recent function
    // Sfloat look_ahead_distance = 4.62281f + 0.00495614f * speed;
    float look_ahead_distance = this->k_dd + 0.00495614f * speed;
    //RCLCPP_INFO(rclcpp::get_logger("speed_to_lookahead"), "kdd=%f", this->k_dd);

    
    return look_ahead_distance;
}

void Pure_Pursuit::set_ekf(geometry_msgs::msg::Pose pose){
    this->current_pose = pose;
}

visualization_msgs::msg::Marker Pure_Pursuit::get_path_marker(){
    return this->path_marker;
}