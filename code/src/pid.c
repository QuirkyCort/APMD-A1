#include "pid.h"

int pid_update(pid_ctrl_t* pid, float measurement, int dt_us) {
    float base = pid->m * pid->setpoint;

    float error = pid->setpoint - measurement;

    // Update integral term with anti-windup
    pid->integral += error * dt_us / 1000000.0;
    if (pid->integral > HIGH_LIMIT) {
        pid->integral = HIGH_LIMIT;
    } else if (pid->integral < LOW_LIMIT) {
        pid->integral = LOW_LIMIT;
    }

    float derivative = (error - pid->previous_error) / (dt_us / 1000000.0);
    pid->previous_error = error;

    int output = (int) (base + pid->kp * error + pid->ki * pid->integral + pid->kd * derivative);

    // Clamp output to limits
    if (output > HIGH_LIMIT) {
        output = HIGH_LIMIT;
    } else if (output < LOW_LIMIT) {
        output = LOW_LIMIT;
    }

    return output;
}