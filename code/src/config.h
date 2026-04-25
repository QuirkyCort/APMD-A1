#ifndef CONFIG_H
#define CONFIG_H

#include "pid.h"

#define MAJOR_VERSION 1
#define MINOR_VERSION 1
#define PATCH_VERSION 1

#define MOTOR_CHANNELS 2
#define SERVO_CHANNELS 2
#define PID_BASE_PERIOD 20000 // 10ms in microseconds

#define MOTOR_CONTROL_LOCK_TIMEOUT_MS 10

#define DEFAULT_PERIOD 1000 // 1ms in microseconds

extern const int PCNT_GPIO[MOTOR_CHANNELS][2];
extern const int MOTOR_GPIO[MOTOR_CHANNELS][2];
extern const int SERVO_PINS[SERVO_CHANNELS];
extern pid_ctrl_t default_speed_pid;
extern pid_ctrl_t default_position_pid;

#endif // CONFIG_H 