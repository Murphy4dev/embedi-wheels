#ifndef __EMBEDI_PID_H
#define __EMBEDI_PID_H

struct _pid {
    unsigned char inited;
    int target;
    float e_k;
    float e_k_1;
    union {
        float e_sum;
        float e_k_2;
    } e;
    int P;
    int I;
    int D;
};
void embedi_pid_init(struct _pid *pid, int t, int p, int i, int d);
float embedi_pid(struct _pid *pid, float current);
float embedi_delta_pid(struct _pid *pid, float current);
#endif
