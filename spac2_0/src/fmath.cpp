#include "spac2_0/fmath.h"
#include "spac2_0/utils.h"

using namespace std;

Pure_Pursuit::Pure_Pursuit(float k_dd)
{
    this->k_dd = k_dd;
}

Pure_Pursuit::Pure_Pursuit()
{
    // TODO: Change to CONSTANT variable MAYBE INCREMENT THIS PARAMETER
    this->k_dd = 1.0;
}

float Pure_Pursuit::calculate_steering_angle(nav_msgs::msg::Path path, float speed)
{
    // Create a pose with the current position  of the car.
    array<float, 2> position = {0.0, 0.0};

    // adds the current position to an array with all the points of the path
    // discarding Z axis
    vector<array<float, 2>> path_points;
    path_points.push_back(position);
    for (long unsigned int i = 0; i < path.poses.size(); i++)
    {
        //CREATE AN ARRAY WITH X AND Y POSITION OF THE PATH, SHIFTING THE X VALUE TO THE REAR OF THE CAR
        array<float, 2> point = {(float) (path.poses[i].pose.position.x + DEFAULT_IMU_TO_REAR_AXLE), (float) path.poses[i].pose.position.y};
        path_points.push_back(point);
    }

    // Calculate look ahead point based on the speed with a min and max distances
    float look_ahead_distance = clamp(speed * k_dd, MIN_LOOKAHEAD, MAX_LOOKAHEAD);

    // Find the closest point to the look ahead distance intersecting the path with a circle
    optional<array<float, 2>> closest_point = get_closest_point(path_points, look_ahead_distance);
    // if there is no intersection with the path, keep the car straight (?) TODO: check if this is the best approach
    if (!closest_point.has_value())
    {
        return 0.0f;
    }
    //if the x value is 0 (straight line) return 0 (no steering angle needed) or else it will give the wrong angle in the atan2
    if ((*closest_point)[0] == 0)
    {
        return 0.0f;
    }

    // Calculate angle between the closest point and (0,0) (because the point is returned relative to (0,0)) instead of the rear!!

    float alpha = atan2((*closest_point)[1], (*closest_point)[0]);

    // Calculate steering angle (pure pursuit algorithm)
    float steering_angle = atan2(2 * WHEELBASE * sin(alpha), look_ahead_distance);

    return steering_angle * 180 / M_PI;
}

PID_Controller::PID_Controller(float min, float max)
{
    PID_Controller();
    min_signal_value = min;
    max_signal_value = max;
}

PID_Controller::PID_Controller()
{
    kp = 0;
    ki = 0;
    kd = 0;
    error = 0;
    error_prev = 0;
    error_sum = 0;
    output_past = 0;
}


float PID_Controller::compute(float setpoint, float input)
{
    error = setpoint - input;
    error_sum += error;
    float output = kp * error + ki * error_sum + kd * (error - error_prev);
    //output_past = output;
    
    if(output > max_signal_value){
        output = max_signal_value;
    }else if(output < min_signal_value){
        output = min_signal_value;
    }else{
        //only update the error if the output is within the limits
        error_prev = error;
    }
    return output;
}

int PID_Controller::set_Tunings(float kp, float ki, float kd)
{
    kp = kp;
    ki = ki;
    kd = kd;
    return 0;
}
float PID_Controller::get_Proportion()
{
    return kp;
}
float PID_Controller::get_Integral()
{
    return ki;
}
float PID_Controller::get_Derivative()
{
    return kd;
}

optional<array<float, 2>> get_closest_point(vector<array<float, 2>> path_points, float look_ahead_distance)
{
    if (path_points.size() > 1)
    {
        for (long unsigned int i = 0; i < path_points.size()-1; i++)
        {
            array<float, 2> point1 = path_points[i];
            array<float, 2> point2 = path_points[i + 1];
            auto intersections = get_intersection(point1, point2, look_ahead_distance);
            // TODO: CHECK IF IT MAKES SENSE TO MAKE THE Y ALWAYS POSITIVE OR IF IT CAN BE NEGATIVE
            //if there is an intersection and the x value is positive
            if (intersections.has_value())
            {
                for (auto intersection : intersections.value())
                {
                    if (intersection[0] > 0)
                    {
                        return intersection;
                    }
                }
            }
        }
    }
    return nullopt;
}


//Function from https://stackoverflow.com/a/59582674/2609987 with some modifications
optional<vector<array<float, 2>>> get_intersection(array<float, 2> point1, array<float, 2> point2, float radius)
{
    array<float, 2> circle_center = {0.0, 0.0};
    vector<array<float,2>> intersections;
    float x1 = point1[0]-circle_center[0];
    float y1 = point1[1]-circle_center[1];
    float x2 = point2[0]-circle_center[0];
    float y2 = point2[1]-circle_center[1];
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dr = sqrt(pow(dx, 2) + pow(dy, 2));
    float D = x1 * y2 - x2 * y1;
    float discriminant = pow(radius, 2) * pow(dr, 2) - pow(D, 2);
    if (discriminant < 0) // there is no intersection
    {
        return nullopt;
    }

    int sign1 = (dy < 0) ? -1 : 1;
    int sign2 = -sign1;

    //TODO: check if everything works out with this axis
    for (int sign : {sign1, sign2}) {
        array<float, 2> temp_intersec;
        temp_intersec[0] = (circle_center[0] + (D * dy + sign * (dy < 0 ? -1 : 1) * dx * sqrt(discriminant))) / (dr * dr);
        temp_intersec[1] = (circle_center[1] + (-D * dx + sign * abs(dy) * sqrt(discriminant))) / (dr * dr);
        intersections.push_back(temp_intersec);
    }

    //filter out intersections that are not within the segment
    vector<float> fraction_along_segment;
    for (auto intersection : intersections) {
        float xi = intersection[0];
        float yi = intersection[1];
        float fraction;
        if (abs(dx) > abs(dy)) {
            fraction = (xi - point1[0]) / dx;
        } else {
            fraction = (yi - point1[1]) / dy;
        }
        fraction_along_segment.push_back(fraction);
    }
    vector<array<float, 2>> filtered_intersections;
    for (long unsigned int i = 0; i < intersections.size(); ++i) {
        float frac = fraction_along_segment[i];
        if (frac >= 0 && frac <= 1) {
            filtered_intersections.push_back(intersections[i]);
        }
    }

    if (filtered_intersections.size() == 2 && abs(discriminant) <= 1e-9) {
        // If the line is tangent to the circle, return just one point
        // (as both intersections have the same location)
        vector<array<float, 2>> tangent_intersection;
        tangent_intersection.push_back(filtered_intersections[0]);
        return tangent_intersection;
    } 
    return filtered_intersections;

}

//function to convert m/s to rpm
int mps_to_rpm(float speed_mps)
{
    return (int) (60 * 4 * (speed_mps / (M_PI * WHEEL_RADIUS * 2)));
}

//function to convert rpm to m/s
float rpm_to_mps(int speed_rpm)
{
    return (float) (((speed_rpm * M_PI * WHEEL_RADIUS * 2) / 60) / 4);
}
