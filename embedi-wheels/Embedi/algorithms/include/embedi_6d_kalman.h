#ifndef __EMBEDI_6D_KALMAN_H
#define __EMBEDI_6D_KALMAN_H

void embedi_6d_kalman_init(void);
void embedi_6d_kalman_filter(float *accel, float *gyro);
struct matrix* emebedi_get_6d_kalman_obsever(void);
void embedi_get_euler_angle(float *angle);
void embedi_quat_to_rot(struct matrix *rot);
#endif
