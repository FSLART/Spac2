#ifndef UTILS_H_
#define UTILS_H_

#define MAX_LOOKAHEAD 12.0f
#define MIN_LOOKAHEAD 5.0f
#define AVG_DISTANCE 0.50f
#define MIN_INDEX 10
//TODO this value needs to be increased for EBS test
#define TERMINAL_RPM 3000.0f
#define DEFAULT_MAX_SPEED 5.0
#define DEFAULT_ACC_SPEED 40.0
#define DEFAULT_EBS_SPEED 40.0

// La grabidad
#define LART_GRAVITY 9.81f

#define DEFAULT_FREQUENCY 100
#define DEFAULT_IMU_TO_REAR_AXLE 1.15

//LOOKAHEAD PARAMETERS
#define DEFAULT_KDD 5.0

// This value is supoded to be used in [0, +inf[
#define DEFAULT_K_CURV 0.001
#define DEFAULT_GRIP_COEF 0.9
#define MAX_SPEED 25.0

// Values for the soft start [0->1.8, 426->2.8, 852->4.4, 1700->11.1]
#define DEFAULT_GROWTH_FACTOR 1.001069
#define DEFAULT_INCREMENT 3.0
#define DEFAULT_LIMITER 25.0

#define SIZE_AVG_ARRAY 3

#endif // UTILS_H_