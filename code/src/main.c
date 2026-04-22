#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "i2c.h"
#include "pid.h"
#include "pcnt.h"
#include "motor.h"
#include "servo.h"

#define MAJOR_VERSION 1
#define MINOR_VERSION 1
#define PATCH_VERSION 1

#define MOTOR_CHANNELS 1
#define PID_BASE_PERIOD 20000 // 10ms in microseconds

const int PCNT_GPIO[MOTOR_CHANNELS][2] = {
    {4, 5},
};

const int MOTOR_GPIO[MOTOR_CHANNELS][2] = {
    {6, 7},
};

const int LED_PINS[] = { 48, 47, 20, 19 };

pid_ctrl_t default_pid = {
    .m = 0.8,
    .kp = 1.0,
    .ki = 1.5,
    .kd = -0.01,
};

motor_t motors[MOTOR_CHANNELS];

static void motor_control_timer_callback(void* arg)
{
    for (int i = 0; i < MOTOR_CHANNELS; i++) {
        int pulse_count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(motors[i].pcnt_unit, &pulse_count));

        motors[i].speed = (pulse_count - motors[i].pulse_count) * 1000000 / PID_BASE_PERIOD; // Convert to pulses per second

        motors[i].power = pid_update(&motors[i].pid, motors[i].speed, PID_BASE_PERIOD);
        motor_set_speed(i, motors[i].power);

        motors[i].pulse_count = pulse_count;
    }
}

void i2c_version_request(i2c_slave_context_t context) {
    uint8_t msg[] = {MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION};
    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, msg, sizeof(msg), &write_len, 1000));
}

void i2c_reset_cmd(i2c_slave_context_t context) {

}

void app_main(void)
{
    i2c_slave_context_t context = init_i2c_slave_context();
 
    // Initialize servo pins
    // servo_init(LED_PINS, sizeof(LED_PINS) / sizeof(LED_PINS[0]));

    // Initialize pulse counter
    for (int i = 0; i < MOTOR_CHANNELS; i++) {
        motors[i].pcnt_unit = pcnt_new(PCNT_GPIO[i][0], PCNT_GPIO[i][1]);
    }

    // Initialize motor control
    motor_init(MOTOR_GPIO, MOTOR_CHANNELS, default_pid, motors);

    // Prep motor update timer
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &motor_control_timer_callback,
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, PID_BASE_PERIOD));

    motors[0].pid.setpoint = 500; // Set desired speed (in pulses per second)

    while (true) {
        i2c_slave_event_t evt;
        if (xQueueReceive(context.event_queue, &evt, 10) == pdTRUE) {
            if (evt == I2C_SLAVE_EVT_RECEIVE) {
                if (context.command == 0x01) { // Reset
                    i2c_reset_cmd(context);
                }

            } else if (evt == I2C_SLAVE_EVT_REQUEST) {
                if (context.command == 0x00) { // Version request
                    i2c_version_request(context);
                }
            }
        }
    }
}

