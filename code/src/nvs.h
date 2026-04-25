#ifndef NVS_H
#define NVS_H

#include <stdio.h>
#include "nvs_flash.h"

#include "config.h"
#include "motor.h"

typedef struct  __attribute__((packed)) {
    float m;
    float kp;
    float ki;
    float kd;
    float limit;
} pid_settings_t;

typedef struct {
    pid_settings_t speed_pid;
    pid_settings_t position_pid;
    uint16_t period;
} motor_settings_t;

typedef struct {
    uint32_t identifier;
    motor_settings_t motor[MOTOR_CHANNELS];
} settings_t;

void init_settings_nvs();
void save_settings_to_nvs(motor_t *motors);
int get_settings_from_nvs(motor_t *motors);

#endif // NVS_H