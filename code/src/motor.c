#include "motor.h"

static const int (*gpios)[2];

// static mcpwm_timer_handle_t timers[6];
// static mcpwm_oper_handle_t operators[6];
// static mcpwm_cmpr_handle_t comparators[6];
// static mcpwm_gen_handle_t generators[6];

void motor_init(const int motor_gpios[][2], int motor_count, motor_t* motors) {
    gpios = motor_gpios;

    // Initialize MCPWM timer
    for (int i = 0; i < motor_count; i++) {
        int group_id = i / 3; // 3 timers per group
        mcpwm_timer_config_t timer_config = {
            .group_id = group_id,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000, // 1 MHz resolution (1 us per tick)
            .period_ticks = motors[i].period, // 1 ms period
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motors[i].timer));
        mcpwm_operator_config_t operator_config = {
            .group_id = group_id, // operator must be in the same group to the timer
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motors[i].operator));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motors[i].operator, motors[i].timer));

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(motors[i].operator, &comparator_config, &motors[i].comparator));

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = gpios[i][0],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].operator, &generator_config, &motors[i].generator));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motors[i].comparator, 0)); // default duty cycle 0%

        // high on timer empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motors[i].generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)
        ));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motors[i].generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motors[i].comparator, MCPWM_GEN_ACTION_HIGH)
        ));

        gpio_reset_pin(gpios[i][1]);
        gpio_set_direction(gpios[i][1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpios[i][1], 1);

        ESP_ERROR_CHECK(mcpwm_timer_enable(motors[i].timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(motors[i].timer, MCPWM_TIMER_START_NO_STOP));
    }
}

void motor_set_dc(int channel, motor_t *motor, int16_t dc) {
    // Swap generator to the other GPIO pin if direction changed
    if (motor->dc >= 0 && dc < 0) {
        ESP_ERROR_CHECK(mcpwm_del_generator(motor->generator));
        gpio_reset_pin(gpios[channel][0]);
        gpio_set_direction(gpios[channel][0], GPIO_MODE_OUTPUT);
        gpio_set_level(gpios[channel][0], 1);

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = gpios[channel][1],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motor->operator, &generator_config, &motor->generator));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motor->generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)
        ));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motor->generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->comparator, MCPWM_GEN_ACTION_HIGH)
        ));

    } else if (motor->dc < 0 && dc >= 0) {
        ESP_ERROR_CHECK(mcpwm_del_generator(motor->generator));
        gpio_reset_pin(gpios[channel][1]);
        gpio_set_direction(gpios[channel][1], GPIO_MODE_OUTPUT);
        gpio_set_level(gpios[channel][1], 1);

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = gpios[channel][0],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motor->operator, &generator_config, &motor->generator));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motor->generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)
        ));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motor->generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->comparator, MCPWM_GEN_ACTION_HIGH)
        )); 
    }
    motor->dc = dc;

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->comparator, abs(motor->dc * motor->period / COMPARE_MAX)));
}

void motor_set_period(int channel, motor_t *motor, uint16_t period) {
    motor->period = period;
    ESP_ERROR_CHECK(mcpwm_timer_set_period(motor->timer, period));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->comparator, abs(motor->dc * motor->period / COMPARE_MAX)));
}