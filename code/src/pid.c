#include "pid.h"

int pid_update(pid_ctrl_t* pid, float measurement, int dt_us) {
    float base = pid->m * pid->setpoint;

    float error = pid->setpoint - measurement;

    // Update integral term with anti-windup
    pid->integral += error * dt_us / 1000000.0;
    if (pid->integral > pid->limit) {
        pid->integral = pid->limit;
    } else if (pid->integral < -pid->limit) {
        pid->integral = -pid->limit;
    }

    float derivative = (error - pid->previous_error) / (dt_us / 1000000.0);
    pid->previous_error = error;

    int output = (int) (base + pid->kp * error + pid->ki * pid->integral + pid->kd * derivative);

    // Clamp output to limits
    if (output > pid->limit) {
        output = pid->limit;
    } else if (output < -pid->limit) {
        output = -pid->limit;
    }

    return output;
}