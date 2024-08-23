#ifndef UTILS_H_
#define UTILS_H_

#define MAX_LOOKAHEAD 20.0f
#define MIN_LOOKAHEAD 2.0f
//TODO this value needs to be increased for EBS test
#define TERMINAL_RPM 500.0f
#define DEFAULT_DESIRED_SPEED 10.0
#define DEFAULT_FREQUENCY 10
#define DEFAULT_IMU_TO_REAR_AXLE 0.990
//TODO: set the default values for the PID controller
#define DEFAULT_KP_SPEED 0.1
#define DEFAULT_KI_SPEED 0.1
#define DEFAULT_KD_SPEED 0.0
#define DEFAULT_KDD 2.0


#endif // UTILS_H_