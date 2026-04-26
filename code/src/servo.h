#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"
#include "config.h"

#define LEDC_CHANNEL LEDC_CHANNEL_0

typedef enum : uint8_t {
    SERVO_OP_NONE = 0,
    SERVO_OP_RUN_DC = 1,
    SERVO_OP_RUN_TO_DC = 2,
} servo_operating_mode_t;

typedef enum : uint8_t {
    SERVO_UP = 1,
    SERVO_DOWN = 2
} servo_direction_t;


typedef struct {
    servo_operating_mode_t mode;
    servo_direction_t direction;
    uint32_t freq;
    float dc;
    uint32_t target_dc;
    float speed; // Used in RUN_TO_DC to specify the speed of change of duty cycle in units of duty cycle per second
} servo_t;

void servo_init(const int pins[], int pin_count);
void servo_set_dc(int channel, servo_t *servo, float duty);
void servo_set_freq(int channel, servo_t *servo, uint32_t freq);

#endif // SERVO_H