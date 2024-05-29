#ifndef __EMBEDI_IMU_H
#define __EMBEDI_IMU_H

#define EMBEDI_X_GYRO (0x40)
#define EMBEDI_Y_GYRO (0x20)
#define EMBEDI_Z_GYRO (0x10)
#define EMBEDI_XYZ_GYRO (EMBEDI_X_GYRO | EMBEDI_Y_GYRO | EMBEDI_Z_GYRO)
#define EMBEDI_XYZ_ACCEL (0x08)
#define ACCEL_X_TARGET 0
#define ACCEL_Y_TARGET 0
#define ACCEL_Z_TARGET _G
#define GYRO_TARGET 0

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ (200)
#define _G (9806L)
#define ACCEL_X_TARGET 0
#define ACCEL_Y_TARGET 0
#define ACCEL_Z_TARGET _G
#define GYRO_TARGET 0
#define CALI_DATA_LEN 20
#define ANGLE_DIRECTION (-1) // 1 or -1
#define rad2deg (57.2957) //180/pi

struct int_param_s {
    void (*cb)(void);
};

struct imu_bias {
    short x_bias;
    short y_bias;
    short z_bias;
};

struct imu_data {
    long data[3];
};

struct imu_operations {
    int (*init)(struct int_param_s *int_param);
    int (*enable_sensor)(unsigned char sensors);
    int (*configure_fifo)(unsigned char sensors);
    int (*set_sample_rate)(unsigned short rate);
    int (*get_sample_rate)(unsigned short *rate);
    int (*set_gyro_fsr)(unsigned short fsr);
    int (*set_accel_fsr)(unsigned char fsr);
    int (*get_gyro_fsr)(unsigned short *fsr);
    int (*get_accel_fsr)(unsigned char *fsr);
    int (*get_gyro_sens)(float *sens);
    int (*get_accel_sens)(unsigned short *sens);
    int (*read_gyro_data)(short *data, unsigned long *timestamp);
    int (*read_accel_data)(short *data, unsigned long *timestamp);
};

struct imu_chip {
    struct imu_operations *ops;
    struct imu_bias accel_bias;
    struct imu_bias gyro_bias;
    struct imu_data accel_data[CALI_DATA_LEN];
    struct imu_data gyro_data[CALI_DATA_LEN];
    int accel_cali_ready;
    int gyro_cali_ready;
};

void embedi_imu_init(void);
void embedi_imu_enable(void);
void embedi_imu_calibration(void);
void embedi_get_accel_data(float *accel_data);
void embedi_get_gyro_data(float *gyro_data);
void embedi_get_roll_angle(float *angle);
int embedi_update_imu_data_buff(void);
void embedi_register_operations(struct imu_operations *ops);
#endif
