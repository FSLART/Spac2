#ifndef UTILS_H_
#define UTILS_H_

#define MAX_LOOKAHEAD 12.0f
#define MIN_LOOKAHEAD 5.0f
#define AVG_DISTANCE 0.50f
#define MIN_INDEX 10
//TODO this value needs to be increased for EBS test
#define TERMINAL_RPM 1500.0f
#define DEFAULT_MAX_SPEED 5.0
#define DEFAULT_FREQUENCY 10
#define DEFAULT_IMU_TO_REAR_AXLE 0.990
//TODO: set the default values for the PID controller
#define DEFAULT_KP_SPEED 0.1
#define DEFAULT_KI_SPEED 0.1
#define DEFAULT_KD_SPEED 0.0
#define DEFAULT_KDD 5.0

// This value is supoded to be used in [0, +inf[
#define DEFAULT_K_CURV 1.5
#define DEFAULT_K_DIST 10.0
#define MAX_SPEED 10.0

#define SIZE_AVG_ARRAY 3

#endif // UTILS_H_