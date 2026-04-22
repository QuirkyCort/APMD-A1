#ifndef PCNT_H
#define PCNT_H

#include "driver/pulse_cnt.h"

#define PCNT_HIGH_LIMIT 30000
#define PCNT_LOW_LIMIT  -30000

pcnt_unit_handle_t pcnt_new(int gpio_A, int gpio_B);

#endif // PCNT_H