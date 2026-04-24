#include <stdio.h>
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

#define MOTOR_CONTROL_LOCK_TIMEOUT_MS 10

const int PCNT_GPIO[MOTOR_CHANNELS][2] = {
    {4, 5},
};

const int MOTOR_GPIO[MOTOR_CHANNELS][2] = {
    {6, 7},
};

const int LED_PINS[] = { 48, 47, 20, 19 };

pid_ctrl_t default_speed_pid = {
    .m = 0.8,
    .kp = 1.0,
    .ki = 1.5,
    .kd = -0.01,
};

pid_ctrl_t default_position_pid = {
    .m = 0,
    .kp = 1.0,
    .ki = 1.5,
    .kd = -0.01,
};

motor_t motors[MOTOR_CHANNELS];
SemaphoreHandle_t motors_write_locks[MOTOR_CHANNELS];

static void motor_control_timer_callback(void* arg)
{
    for (int i = 0; i < MOTOR_CHANNELS; i++) {
        // Calculate speed
        int pulse_count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(motors[i].pcnt_unit, &pulse_count));
        motors[i].speed = (pulse_count - motors[i].pulse_count) * 1000000 / PID_BASE_PERIOD; // Convert to pulses per second
        motors[i].pulse_count = pulse_count;

        // Update motor control based on operating mode
        if (motors[i].mode == MOTOR_OP_RUN_DC) {
            // motor_set_dc should be called by the function that sets the DC, so we don't need to do anything here
            continue;
        } else if (motors[i].mode == MOTOR_OP_RUN_SPEED) {
            int dc = pid_update(&motors[i].speed_pid, motors[i].speed, PID_BASE_PERIOD);
            motor_set_dc(i, &motors[i], dc);
        }

    }
}

void i2c_version_request(i2c_slave_context_t context) {
    uint8_t msg[] = {MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION};
    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, msg, sizeof(msg), &write_len, 1000));
}

void i2c_reset_cmd(i2c_slave_context_t context) {

}

void i2c_get_speed_pid(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    struct {
        float m;
        float kp;
        float ki;
        float kd;
    } __attribute__((packed)) msg;

    msg.m = motors[channel].speed_pid.m;
    msg.kp = motors[channel].speed_pid.kp;
    msg.ki = motors[channel].speed_pid.ki;
    msg.kd = motors[channel].speed_pid.kd;

    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, (uint8_t*) &msg, sizeof(msg), &write_len, 1000));
}

void i2c_set_speed_pid(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    struct __attribute__((packed)) msg {
        float m;
        float kp;
        float ki;
        float kd;
    };

    if (context.length != 1 + sizeof(struct msg)) {
        return;
    }

    struct msg *msg = (struct msg *) (context.buffer + 1);

    motors[channel].speed_pid.m = msg->m;
    motors[channel].speed_pid.kp = msg->kp;
    motors[channel].speed_pid.ki = msg->ki;
    motors[channel].speed_pid.kd = msg->kd;
}

void i2c_get_position_pid(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    struct {
        float m;
        float kp;
        float ki;
        float kd;
    } __attribute__((packed)) msg;

    msg.m = motors[channel].position_pid.m;
    msg.kp = motors[channel].position_pid.kp;
    msg.ki = motors[channel].position_pid.ki;
    msg.kd = motors[channel].position_pid.kd;

    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, (uint8_t*) &msg, sizeof(msg), &write_len, 1000));
}

void i2c_set_position_pid(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    struct __attribute__((packed)) msg {
        float m;
        float kp;
        float ki;
        float kd;
    };

    if (context.length != 1 + sizeof(struct msg)) {
        return;
    }

    struct msg *msg = (struct msg *) (context.buffer + 1);

    motors[channel].position_pid.m = msg->m;
    motors[channel].position_pid.kp = msg->kp;
    motors[channel].position_pid.ki = msg->ki;
    motors[channel].position_pid.kd = msg->kd;
}

void i2c_get_pwm_period(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, (uint8_t*) &motors[channel].period, sizeof(motors[channel].period), &write_len, 1000));
}

void i2c_set_pwm_period(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    if (context.length != 1 + sizeof(int)) {
        return;
    }

    int *period = (int *) (context.buffer + 1);
    motor_set_period(channel, &motors[channel], *period);
}

void i2c_get_stop_mode(i2c_slave_context_t context) {
    int channel = context.buffer[1];
    
    uint8_t stop_mode = motors[channel].stop_mode;
    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, &stop_mode, sizeof(stop_mode), &write_len, 1000));
}

void i2c_set_stop_mode(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    if (context.length != 2) {
        return;
    }

    uint8_t stop_mode = context.buffer[1];
    motors[channel].stop_mode = stop_mode;
}

void i2c_get_dc(i2c_slave_context_t context) {
    int channel = context.buffer[1];
    
    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, (uint8_t*) &motors[channel].dc, sizeof(motors[channel].dc), &write_len, 1000));
}

void i2c_set_dc(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    if (context.length != 1 + sizeof(int16_t)) {
        return;
    }

    xSemaphoreTake(motors_write_locks[channel], pdTICKS_TO_MS(MOTOR_CONTROL_LOCK_TIMEOUT_MS));
    motors[channel].mode = MOTOR_OP_RUN_DC;
    motor_set_dc(channel, &motors[channel], * (int16_t *) (context.buffer + 1));
    xSemaphoreGive(motors_write_locks[channel]);
}

void i2c_get_target_speed(i2c_slave_context_t context) {
    int channel = context.buffer[1];
    
    uint32_t write_len;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_write(context.handle, (uint8_t*) &motors[channel].speed_pid.setpoint, sizeof(motors[channel].speed_pid.setpoint), &write_len, 1000));
}

void i2c_set_target_speed(i2c_slave_context_t context) {
    int channel = context.buffer[1];

    if (context.length != 1 + sizeof(float)) {
        return;
    }

    xSemaphoreTake(motors_write_locks[channel], pdTICKS_TO_MS(MOTOR_CONTROL_LOCK_TIMEOUT_MS));
    motors[channel].mode = MOTOR_OP_RUN_SPEED;
    motors[channel].speed_pid.setpoint = *(float *) (context.buffer + 1);
    xSemaphoreGive(motors_write_locks[channel]);
}

void app_main(void)
{
    i2c_slave_context_t context = init_i2c_slave_context();

    // Setup semaphores for motor control
    for (int i = 0; i < MOTOR_CHANNELS; i++) {
        motors_write_locks[i] = xSemaphoreCreateBinary();
        xSemaphoreGive(motors_write_locks[i]);
    }
 
    // Initialize servo pins
    // servo_init(LED_PINS, sizeof(LED_PINS) / sizeof(LED_PINS[0]));

    // Initialize pulse counter
    for (int i = 0; i < MOTOR_CHANNELS; i++) {
        motors[i].pcnt_unit = pcnt_new(PCNT_GPIO[i][0], PCNT_GPIO[i][1]);
    }

    // Initialize motor control
    motor_init(MOTOR_GPIO, MOTOR_CHANNELS, default_speed_pid, default_position_pid, motors);

    // Prep motor update timer
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &motor_control_timer_callback,
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, PID_BASE_PERIOD));

    while (true) {
        i2c_slave_event_t evt;
        if (xQueueReceive(context.event_queue, &evt, 10) == pdTRUE) {
            if (evt == I2C_SLAVE_EVT_RECEIVE) {
                switch (context.command) {
                    case VERSION_REGISTER:
                        if (context.length == 2) {
                            i2c_version_request(context);
                        }
                        break;
                    case RESET_REGISTER:
                        i2c_reset_cmd(context);
                        break;
                    case SPEED_PID_REGISTER:
                        if (context.length == 2) {
                            i2c_get_speed_pid(context);
                        } else {
                            i2c_set_speed_pid(context);
                        }
                        break;
                    case POSITION_PID_REGISTER:
                        if (context.length == 2) {
                            i2c_get_position_pid(context);
                        } else {
                            i2c_set_position_pid(context);
                        }
                        break;
                    case PWM_PERIOD_REGISTER:
                        if (context.length == 2) {
                            i2c_get_pwm_period(context);
                        } else {
                            i2c_set_pwm_period(context);
                        }
                        break;
                    case STOP_MODE_REGISTER:
                        if (context.length == 2) {
                            i2c_get_stop_mode(context);
                        } else {
                            i2c_set_stop_mode(context);
                        }
                        break;
                    case DC_REGISTER:
                        if (context.length == 2) {
                            i2c_get_dc(context);
                        } else {
                            i2c_set_dc(context);
                        }
                        break;
                    case TARGET_SPEED_REGISTER:
                        if (context.length == 2) {
                            i2c_get_target_speed(context);
                        } else {
                            i2c_set_target_speed(context);
                        }
                        break;
                }
            }
        }
    }
}

