#ifndef MOTOR_H
#define MOTOR_H

#include <string.h>
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "pid.h"
#include "pcnt.h"

#define COMPARE_MAX 1000

typedef enum {
    MOTOR_OP_NONE,
    MOTOR_OP_RUN_DC,
    MOTOR_OP_RUN_SPEED,
    MOTOR_OP_HOLD,
} motor_operating_mode_t;

typedef enum {
    MOTOR_STOP_BRAKE = 1,
    MOTOR_STOP_HOLD = 2,
} motor_stop_mode_t;

typedef struct {
    motor_operating_mode_t mode;
    motor_stop_mode_t stop_mode;
    pid_ctrl_t pid;
    pcnt_unit_handle_t pcnt_unit;
    int pulse_count;
    int speed; // Actual, not set point
    int power; // -1000 to 1000, where 1000 is full forward and -1000 is full reverse
    int max_speed; // Used when running with ramp
    int target_position;
} motor_t;


void motor_init(const int motor_gpios[][2], int motor_count, pid_ctrl_t default_pid, motor_t* motors);
void motor_set_speed(int channel, int speed);

#endif // MOTOR_H