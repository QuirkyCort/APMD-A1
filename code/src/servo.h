#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"


void servo_init(const int pins[], int pin_count);
void servo_set_dc(int channel, int duty);
void servo_set_freq(int channel, int freq);
uint32_t servo_get_freq(int channel);

#endif // SERVO_H