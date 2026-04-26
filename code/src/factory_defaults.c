#include "config.h"

const int PCNT_GPIO[MOTOR_CHANNELS][2] = {
    {4, 5},
    {8, 9},
};

const int MOTOR_GPIO[MOTOR_CHANNELS][2] = {
    {6, 7},
    {10, 11},
};

const int SERVO_PINS[] = { 17, 18 };

pid_ctrl_t default_speed_pid = {
    .m = 0.8,
    .kp = 1.0,
    .ki = 1.5,
    .kd = -0.01,
    .limit = 1000.0,
};

pid_ctrl_t default_position_pid = {
    .m = 0.0,
    .kp = 16.0,
    .ki = 0.0,
    .kd = 0.0,
    .limit = 1500.0,
};
