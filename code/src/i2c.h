#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_slave.h"

#define I2C_SLAVE_SCL_IO 15
#define I2C_SLAVE_SDA_IO 16
#define I2C_SLAVE_NUM    0
#define ESP_SLAVE_ADDR   0x56

// We use a 2 bytes register address, the first byte is the register and the
// second byte refers to the motor / servo channel (...or ignored where not applicable)
#define VERSION_REGISTER 0x00
#define RESET_REGISTER 0x01
#define SAVE_SETTINGS_REGISTER 0x02
#define SPEED_PID_REGISTER 0x10
#define POSITION_PID_REGISTER 0x11
#define PWM_PERIOD_REGISTER 0x12
#define STOP_MODE_REGISTER 0x13
#define DC_REGISTER 0x14
#define TARGET_SPEED_REGISTER 0x15
#define SPEED_REGISTER 0x16
#define STEPS_REGISTER 0x17
#define CLEAR_STEPS_REGISTER 0x18
#define TARGET_POSITION_REGISTER 0x19
#define SERVO_FREQ_REGISTER 0x20
#define SERVO_DC_REGISTER 0x21

typedef enum {
    I2C_SLAVE_EVT_REQUEST,
    I2C_SLAVE_EVT_RECEIVE
} i2c_slave_event_t;

typedef struct {
    uint8_t command;
    char buffer[32];
    int length;
    QueueHandle_t event_queue;
    i2c_slave_dev_handle_t handle;
} i2c_slave_context_t;

int init_i2c_slave_context(i2c_slave_context_t *context);

#endif // I2C_H