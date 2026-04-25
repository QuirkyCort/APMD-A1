#include "nvs.h"

nvs_handle_t nvs_default_partition_handle;

void init_settings_nvs() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_open("settings", NVS_READWRITE, &nvs_default_partition_handle));
}

void save_settings_to_nvs(motor_t *motors) {
    settings_t settings;
    settings.identifier = 0x56010101; // Semi-random identifier to identify valid settings

    for (int i = 0; i < MOTOR_CHANNELS; i++) {
        settings.motor[i].speed_pid.m = motors[i].speed_pid.m;
        settings.motor[i].speed_pid.kp = motors[i].speed_pid.kp;
        settings.motor[i].speed_pid.ki = motors[i].speed_pid.ki;
        settings.motor[i].speed_pid.kd = motors[i].speed_pid.kd;
        settings.motor[i].speed_pid.limit = motors[i].speed_pid.limit;
        settings.motor[i].position_pid.m = motors[i].position_pid.m;
        settings.motor[i].position_pid.kp = motors[i].position_pid.kp;
        settings.motor[i].position_pid.ki = motors[i].position_pid.ki;
        settings.motor[i].position_pid.kd = motors[i].position_pid.kd;
        settings.motor[i].position_pid.limit = motors[i].position_pid.limit;
        settings.motor[i].period = motors[i].period;
    }
    ESP_ERROR_CHECK(nvs_set_blob(nvs_default_partition_handle, "settings", &settings, sizeof(settings)));
    ESP_ERROR_CHECK(nvs_commit(nvs_default_partition_handle));
}

int get_settings_from_nvs(motor_t *motors) {
    settings_t settings;
    size_t size;
    ESP_ERROR_CHECK(nvs_get_blob(nvs_default_partition_handle, "settings", &settings, &size));
    if (settings.identifier != 0x56010101) {
        return -1; // No valid settings found
    }
    for (int i = 0; i < MOTOR_CHANNELS; i++) {
        motors[i].speed_pid.m = settings.motor[i].speed_pid.m;
        motors[i].speed_pid.kp = settings.motor[i].speed_pid.kp;
        motors[i].speed_pid.ki = settings.motor[i].speed_pid.ki;
        motors[i].speed_pid.kd = settings.motor[i].speed_pid.kd;
        motors[i].speed_pid.limit = settings.motor[i].speed_pid.limit;
        motors[i].position_pid.m = settings.motor[i].position_pid.m;
        motors[i].position_pid.kp = settings.motor[i].position_pid.kp;
        motors[i].position_pid.ki = settings.motor[i].position_pid.ki;
        motors[i].position_pid.kd = settings.motor[i].position_pid.kd;
        motors[i].position_pid.limit = settings.motor[i].position_pid.limit;
        motors[i].period = settings.motor[i].period;
    }
    return 0; // Success
}