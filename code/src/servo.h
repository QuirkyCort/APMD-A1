#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"
#include "config.h"

#define LEDC_CHANNEL            LEDC_CHANNEL_0


void servo_init(const int pins[], int pin_count);
void servo_set_dc(int channel, int duty);
void servo_set_freq(int channel, int freq);
uint32_t servo_get_freq(int channel);

#endif // SERVO_H