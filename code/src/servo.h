#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"
#include "config.h"

#define LEDC_CHANNEL            LEDC_CHANNEL_0

typedef struct {
    uint32_t freq;
    uint32_t dc;
} servo_t;

void servo_init(const int pins[], int pin_count);
void servo_set_dc(int channel, servo_t *servo, uint32_t duty);
void servo_set_freq(int channel, servo_t *servo, uint32_t freq);

#endif // SERVO_H