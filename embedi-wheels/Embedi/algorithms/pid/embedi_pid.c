#include "embedi_pid.h"
#include "embedi_config.h"
#include <stdio.h>
#include <string.h>

void embedi_pid_init(struct _pid *pid, float t, float p, float i, float d)
{
    if (!pid) {
        printf("pid null \n");
        return;
    }

    memset(pid, 0, sizeof(struct _pid));
    pid->P = p;
    pid->I = i;
    pid->D = d;
    pid->e_k = 0;
    pid->e_k_1 = 0;
    pid->e.e_k_2 = 0;
    pid->target = t;
    pid->inited = 1;
}

void embedi_pid_reset(struct _pid *pid)
{
    if (!pid) {
        printf("pid null \n");
        return;
    }

    pid->e_k = 0;
    pid->e_k_1 = 0;
    pid->e.e_k_2 = 0;
}

void embedi_pid_change(struct _pid *pid, float p, float i, float d)
{
    if (!pid) {
        printf("pid null \n");
        return;
    }
    pid->P = p;
    pid->I = i;
    pid->D = d;
}

void embedi_change_pid_target(struct _pid *pid, float t)
{
    if (!pid) {
        printf("pid null \n");
        return;
    }
    pid->target = t;
}

float embedi_pid(struct _pid *pid, float current, float i_limit)
{
    float data = 0;

    if (!pid || !pid->inited) {
        printf("pid not inited \n");
        return 0;
    }
    pid->e_k_1 = pid->e_k;
    pid->e_k = pid->target - current;
    pid->e.e_sum += pid->e_k;

    //i limit
    if (pid->e.e_sum > i_limit)
        pid->e.e_sum = i_limit;
    else if (pid->e.e_sum < -i_limit) 
        pid->e.e_sum = -i_limit;

    data = pid->P * (pid->e_k) +
           pid->I * (pid->e.e_sum) +
           pid->D * (pid->e_k - pid->e_k_1);
    // printf("data %d e_k %d w %d \n", (int)data,
    //        (int)(pid->e_k * 1000), (int)((pid->e_k - pid->e_k_1) * 1000));

    return data;
}

float embedi_delta_pid(struct _pid *pid, float current)
{
    float delta = 0;

    if (!pid || !pid->inited) {
        printf("pid not inited \n");
        return 0;
    }
    pid->e.e_k_2 = pid->e_k_1;
    pid->e_k_1 = pid->e_k;
    pid->e_k = pid->target - current;

    delta = pid->P * (pid->e_k - pid->e_k_1) +
            pid->I * (pid->e_k) +
            pid->D * (pid->e_k - 2 * pid->e_k_1 - pid->e.e_k_2);
    return delta;
}
