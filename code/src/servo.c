#include "servo.h"

void servo_init(const int pins[], int pin_count)
{
    for (int i = 0; i < pin_count; i++) {
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_12_BIT,
            .timer_num        = i,
            .freq_hz          = SERVO_DEFAULT_FREQ, // uint32_t
            .clk_cfg          = LEDC_USE_RC_FAST_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = i,
            .timer_sel      = i,
            .gpio_num       = pins[i],
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

void servo_set_dc(int channel, servo_t *servo, uint32_t duty) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
    servo->dc = duty;
}

void servo_set_freq(int channel, servo_t *servo, uint32_t freq) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_set_freq(LEDC_LOW_SPEED_MODE, channel, freq));
    servo->freq = freq;
}
