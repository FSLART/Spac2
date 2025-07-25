#ifndef UTILS_H_
#define UTILS_H_

#define MAX_LOOKAHEAD 12.0f
#define MIN_LOOKAHEAD 5.0f
#define AVG_DISTANCE 0.50f
#define MIN_INDEX 10

// SPEED PARAMETERS
#define TERMINAL_RPM 3000.0f
#define DEFAULT_MAX_SPEED 5.0
#define DEFAULT_ACC_SPEED 40.0
#define DEFAULT_EBS_SPEED 40.0

//MISC
#define DEFAULT_FREQUENCY 100
#define DEFAULT_IMU_TO_REAR_AXLE 1.15f

//LOOKAHEAD PARAMETERS
#define DEFAULT_KDD 5.0

// This value is supoded to be used in [0, +inf[
#define DEFAULT_K_CURV 0.5
#define DEFAULT_K_DIST 8.0
#define MAX_SPEED 5.0

// Values for the soft start [0->1.8, 426->2.8, 852->4.4, 1700->11.1]
#define DEFAULT_GROWTH_FACTOR 1.000951
#define DEFAULT_INCREMENT 4.0
#define DEFAULT_ACC_INCREMENT 7.0
#define DEFAULT_LIMITER 25.0

#define SIZE_AVG_ARRAY 3

#endif // UTILS_H_