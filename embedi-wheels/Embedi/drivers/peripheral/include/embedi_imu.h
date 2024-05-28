#ifndef __EMBEDI_IMU_H
#define __EMBEDI_IMU_H

void embedi_imu_init(void);
void embedi_imu_enable(void);
void embedi_imu_calibration(void);
void embedi_get_accel_data(float *accel_data);
void embedi_get_gyro_data(float *gyro_data);
void embedi_get_roll_angle(float *angle);
int embedi_update_imu_data_buff(void);
#endif
