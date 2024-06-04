#ifndef __EMBEDI_PID_H
#define __EMBEDI_PID_H

struct _pid {
    unsigned char inited;
    float target;
    float e_k;
    float e_k_1;
    union {
        float e_sum;
        float e_k_2;
    } e;
    float P;
    float I;
    float D;
};
void embedi_pid_init(struct _pid *pid, float t, float p, float i, float d);
void embedi_pid_reset(struct _pid *pid);
float embedi_pid(struct _pid *pid, float current, float i_limit);
float embedi_delta_pid(struct _pid *pid, float current);
void embedi_change_pid_target(struct _pid *pid, float t);
void embedi_pid_change(struct _pid *pid, float p, float i, float d);
#endif
