#include "motor.h"

static const int (*gpios)[2];
static int compare_values[] = {0, 0, 0, 0, 0, 0};
static int periods[] = {1000, 1000, 1000, 1000, 1000, 1000};

static mcpwm_timer_handle_t timers[6];
static mcpwm_oper_handle_t operators[6];
static mcpwm_cmpr_handle_t comparators[6];
static mcpwm_gen_handle_t generators[6];

void motor_init(const int motor_gpios[][2], int motor_count, pid_ctrl_t default_pid, motor_t* motors) {
     for (int i = 0; i < motor_count; i++) {
        memcpy(&motors[i].pid, &default_pid, sizeof(pid_ctrl_t));
        motors[i].mode = MOTOR_OP_RUN_SPEED;
        motors[i].stop_mode = MOTOR_STOP_BRAKE;
        motors[i].pulse_count = 0;
        motors[i].speed = 0;
        motors[i].power = 0;
        motors[i].max_speed = 0;
        motors[i].target_position = 0;
    }

    gpios = motor_gpios;

    // Initialize MCPWM timer
    for (int i = 0; i < motor_count; i++) {
        int group_id = i / 3; // 3 timers per group
        mcpwm_timer_config_t timer_config = {
            .group_id = group_id,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000, // 1 MHz resolution (1 us per tick)
            .period_ticks = periods[i], // 1 ms period
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[i]));
        mcpwm_operator_config_t operator_config = {
            .group_id = group_id, // operator must be in the same group to the timer
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timers[i]));

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &comparator_config, &comparators[i]));

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = gpios[i][0],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &generator_config, &generators[i]));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], compare_values[i]));

        // high on timer empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)
        ));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_HIGH)
        ));

        gpio_reset_pin(gpios[i][1]);
        gpio_set_direction(gpios[i][1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpios[i][1], 1);

        ESP_ERROR_CHECK(mcpwm_timer_enable(timers[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[i], MCPWM_TIMER_START_NO_STOP));
    }
}

void motor_set_speed(int channel, int speed) {
    // Swap generator to the other GPIO pin if direction changed
    if (compare_values[channel] >= 0 && speed < 0) {
        ESP_ERROR_CHECK(mcpwm_del_generator(generators[channel]));
        gpio_reset_pin(gpios[channel][0]);
        gpio_set_direction(gpios[channel][0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpios[channel][0], 1);

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = gpios[channel][1],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[channel], &generator_config, &generators[channel]));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generators[channel],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)
        ));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generators[channel],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[channel], MCPWM_GEN_ACTION_HIGH)
        ));

    } else if (compare_values[channel] < 0 && speed >= 0) {
        ESP_ERROR_CHECK(mcpwm_del_generator(generators[channel]));
        gpio_reset_pin(gpios[channel][1]);
        gpio_set_direction(gpios[channel][1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpios[channel][1], 1);

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = gpios[channel][0],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[channel], &generator_config, &generators[channel]));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generators[channel],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)
        ));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generators[channel],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[channel], MCPWM_GEN_ACTION_HIGH)
        )); 
    }

    speed = speed * periods[channel] / COMPARE_MAX; // scale speed to timer period

    compare_values[channel] = speed;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[channel], abs(speed)));
}
