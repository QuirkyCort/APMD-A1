#ifndef MOTOR_H
#define MOTOR_H

#include <string.h>
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "pid.h"
#include "pcnt.h"

#define COMPARE_MAX 1000

typedef enum : uint8_t {
    MOTOR_OP_NONE = 0,
    MOTOR_OP_RUN_DC = 1,
    MOTOR_OP_RUN_SPEED = 2,
    MOTOR_OP_HOLD_POSITION = 3,
    MOTOR_OP_RUN_TO_POSITION = 4,
} motor_operating_mode_t;

typedef enum : uint8_t {
    MOTOR_STOP_BRAKE = 1,
    MOTOR_STOP_HOLD = 2,
} motor_stop_mode_t;

typedef enum : uint8_t {
    MOTOR_STOPPED = 1,
    MOTOR_RUNNING = 2
} motor_status_t;

typedef struct {
    motor_operating_mode_t mode;
    motor_stop_mode_t stop_mode;
    motor_status_t status;
    pid_ctrl_t speed_pid;
    pid_ctrl_t position_pid;
    pcnt_unit_handle_t pcnt_unit;
    uint16_t period; // in microseconds
    int32_t steps;
    int16_t speed; // Actual, not set point. In pulses per second.
    int16_t dc; // -1000 to 1000, where 1000 is full forward and -1000 is full reverse
    uint16_t max_speed; // Used when running with ramp and running to position with speed limit
    int32_t target_position;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
} motor_t;


void motor_init(const int motor_gpios[][2], int motor_count, motor_t* motors);
void motor_set_dc(int channel, motor_t *motor, int16_t dc);
void motor_set_period(int channel, motor_t *motor, uint16_t period);

#endif // MOTOR_H