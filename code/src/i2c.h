#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_slave.h"

#define I2C_SLAVE_SCL_IO 15               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 16               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM    0
#define ESP_SLAVE_ADDR   55           /*!< ESP slave address, you can set any 7bit value */

typedef struct {
    uint8_t command;
    char buffer[10];
    QueueHandle_t event_queue;
    i2c_slave_dev_handle_t handle;
} i2c_slave_context_t;

typedef enum {
    I2C_SLAVE_EVT_REQUEST,
    I2C_SLAVE_EVT_RECEIVE
} i2c_slave_event_t;

i2c_slave_context_t init_i2c_slave_context();

#endif // I2C_H