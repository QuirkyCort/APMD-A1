#ifndef PID_H
#define PID_H

#define HIGH_LIMIT 1000
#define LOW_LIMIT  -1000


typedef struct pid_ctrl_t {
    float m;
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float previous_error;
} pid_ctrl_t;

int pid_update(pid_ctrl_t* pid, float measurement, int dt_us);

#endif // PID_H