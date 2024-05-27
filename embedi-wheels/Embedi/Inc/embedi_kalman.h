#ifndef __EMBEDI_KALMAN_H
#define __EMBEDI_KALMAN_H

void embedi_kalman_init(void);
void embedi_kalman_filter(float accel_y, float accel_z, float gyro);

#endif
