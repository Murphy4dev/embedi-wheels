#ifndef __EMBEDI_KALMAN_H
#define __EMBEDI_KALMAN_H

void embedi_kalman_init(void);
void embedi_kalman_filter(float accel_angle, float gyro);
struct matrix* emebedi_get_kalman_estimation(void);
#endif
