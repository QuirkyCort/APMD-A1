#ifndef PID_H
#define PID_H


typedef struct pid_ctrl_t {
    float m;
    float kp;
    float ki;
    float kd;
    float limit;
    float setpoint;
    float integral;
    float previous_error;
} pid_ctrl_t;

int pid_update(pid_ctrl_t* pid, float measurement, int dt_us);

#endif // PID_H