#ifndef __EMBEDI_2D_KALMAN_H
#define __EMBEDI_2D_KALMAN_H

void embedi_2d_kalman_init(void);
void embedi_2d_kalman_filter(float accel_angle, float gyro);
struct matrix* emebedi_get_2d_kalman_obsever(void);
#endif
