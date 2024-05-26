#include "embedi_config.h"
#include "embedi_math.h"
#include <stdio.h>

void embedi_kalman_init(void)
{

}
#if 0
struct _kalman {
    float dt;
    float R;
    float Q;
};

static struct _kalman filter = {
    .dt = 0.005,
    .Q = 0.1,
    .R = 0.001,
};

static void _predect(float accel, float gyro)
{
}

static void _update(void)
{
}
#endif
