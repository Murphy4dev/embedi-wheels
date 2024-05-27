#include "embedi_config.h"
#include "embedi_test.h"
#include <stdio.h>
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#endif
#include "embedi_flash.h"
#include "embedi_i2c.h"
#include "embedi_kalman.h"
#include "embedi_math.h"
#include "embedi_scope.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ (200)
#define _G (9806L)
#define ACCEL_X_TARGET 0
#define ACCEL_Y_TARGET 0
#define ACCEL_Z_TARGET _G
#define GYRO_TARGET 0
#define CALI_DATA_LEN 20
#define ANGLE_DIRECTION (-1) // 1 or -1

extern int run_test;
static void _read_from_flash(void);

void embedi_imu_init(void)
{
    struct int_param_s int_param;
    unsigned short gyro_rate, gyro_fsr;
    unsigned char accel_fsr;
    int result = 0;

    result = mpu_init(&int_param);
    if (result) {
        printf("Could not initialize imu.\n");
    }
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);

    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    printf("imu config %d %d %d \n", gyro_rate, gyro_fsr, accel_fsr);
    _read_from_flash();

    mpu_set_sensors(0);
    embedi_kalman_init();
}

void embedi_imu_enable(void)
{
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
}

struct imu_bias {
    short x_bias;
    short y_bias;
    short z_bias;
};
static struct imu_bias accel_bias;
static struct imu_bias gyro_bias;

struct imu_data {
    long data[3];
};
static struct imu_data accel_data[CALI_DATA_LEN];
static struct imu_data gyro_data[CALI_DATA_LEN];
static uint8_t accel_cali_ready = 0;
static uint8_t gyro_cali_ready = 0;

static void _accel_data_collection(long *accel)
{
    static int index = 0;

    accel_data[index % CALI_DATA_LEN].data[0] = accel[0];
    accel_data[index % CALI_DATA_LEN].data[1] = accel[1];
    accel_data[index % CALI_DATA_LEN].data[2] = accel[2];
    index++;

    if (index == CALI_DATA_LEN) {
        index = 0;
        accel_cali_ready = 1;
    }
}

static void _gyro_data_collection(long *gyro)
{
    static int index = 0;

    gyro_data[index % CALI_DATA_LEN].data[0] = gyro[0];
    gyro_data[index % CALI_DATA_LEN].data[1] = gyro[1];
    gyro_data[index % CALI_DATA_LEN].data[2] = gyro[2];
    index++;

    if (index == CALI_DATA_LEN) {
        index = 0;
        gyro_cali_ready = 1;
    }
}
#define ACCEL_X_TARGET 0
#define ACCEL_Y_TARGET 0
#define ACCEL_Z_TARGET _G
#define GYRO_TARGET 0
static void _accel_callibratin(void)
{
    int index = 0;
    unsigned short accel_sens = 0;
    int x_sum, y_sum, z_sum;
    int x_avg, y_avg, z_avg;

    if (!accel_cali_ready) {
        printf("accel cali collection not ready \n");
        return;
    }
    x_sum = y_sum = z_sum = 0;
    x_avg = y_avg = z_avg = 0;

    for (index = 0; index < CALI_DATA_LEN; index++) {
        x_sum += accel_data[index].data[0];
        y_sum += accel_data[index].data[1];
        z_sum += accel_data[index].data[2];
        printf("acc cali:[%d, %d, %d]\n",
               accel_data[index].data[0], accel_data[index].data[1],
               accel_data[index].data[2]);
    }

    x_avg = x_sum / CALI_DATA_LEN;
    y_avg = y_sum / CALI_DATA_LEN;
    z_avg = z_sum / CALI_DATA_LEN;
    printf("acc avg:[%d, %d, %d]\n", x_avg, y_avg, z_avg);

    mpu_get_accel_sens(&accel_sens);
    accel_bias.x_bias = (ACCEL_X_TARGET * accel_sens / _G - x_avg);
    accel_bias.y_bias = (ACCEL_Y_TARGET * accel_sens / _G - y_avg);
    accel_bias.z_bias = (ACCEL_Z_TARGET * accel_sens / _G - z_avg);
    printf("acc z_target_raw:[%d] accel_sens:[%d]\n",
           ACCEL_Z_TARGET * accel_sens / _G, accel_sens);
    printf("acc bias:[%d, %d, %d]\n",
           accel_bias.x_bias, accel_bias.y_bias,
           accel_bias.z_bias);
}

static void _gyro_callibratin(void)
{
    int index = 0;
    int x_sum, y_sum, z_sum;
    int x_avg, y_avg, z_avg;

    if (!gyro_cali_ready) {
        printf("gyro cali collection not ready \n");
        return;
    }

    x_sum = y_sum = z_sum = 0;
    x_avg = y_avg = z_avg = 0;

    for (index = 0; index < CALI_DATA_LEN; index++) {
        x_sum += gyro_data[index].data[0];
        y_sum += gyro_data[index].data[1];
        z_sum += gyro_data[index].data[2];
        printf("gyro cali:[%d, %d, %d]\n",
               gyro_data[index].data[0], gyro_data[index].data[1],
               gyro_data[index].data[2]);
    }

    x_avg = x_sum / CALI_DATA_LEN;
    y_avg = y_sum / CALI_DATA_LEN;
    z_avg = z_sum / CALI_DATA_LEN;
    printf("gyro avg:[%d, %d, %d]\n", x_avg, y_avg, z_avg);

    gyro_bias.x_bias = GYRO_TARGET - x_avg;
    gyro_bias.y_bias = GYRO_TARGET - y_avg;
    gyro_bias.z_bias = GYRO_TARGET - z_avg;

    printf("gyro bias:[%d, %d, %d]\n",
           gyro_bias.x_bias, gyro_bias.y_bias,
           gyro_bias.z_bias);
}

static void _accel_data_standardize(long *data, float *accel)
{
    unsigned short accel_sens;

    if (!accel || !data)
        return;
    mpu_get_accel_sens(&accel_sens);
    accel[0] = (float)data[0] / accel_sens * 9.8;
    accel[1] = (float)data[1] / accel_sens * 9.8;
    accel[2] = (float)data[2] / accel_sens * 9.8;
#ifdef CFG_IMU_DATA_SCOPE_SHOW
#if (ACCEL_DATA_SHOW == 1)
    embedi_data_to_scope(accel[0], CHANNEL_1);
    embedi_data_to_scope(accel[1], CHANNEL_2);
    embedi_data_to_scope(accel[2], CHANNEL_3);
    embedi_scope_show();
#endif
#else
    // printf("accel data %f %f %f \n", accel[0], accel[1], accel[2]);
#endif
}

static void _gyro_data_standardize(long *data, float *gyro)
{
    float gyro_sens;

    if (!gyro || !data)
        return;

    mpu_get_gyro_sens(&gyro_sens);
    gyro[0] = data[0] / gyro_sens * ((2 * 3.1415926) / 360);
    gyro[1] = data[1] / gyro_sens * ((2 * 3.1415926) / 360);
    gyro[2] = data[2] / gyro_sens * ((2 * 3.1415926) / 360);
#ifdef CFG_IMU_DATA_SCOPE_SHOW
#if (GRYO_DATA_SHOW == 1)
    embedi_data_to_scope(gyro[0], CHANNEL_1);
    embedi_data_to_scope(gyro[1], CHANNEL_2);
    embedi_data_to_scope(gyro[2], CHANNEL_3);
    embedi_scope_show();
#endif
#else
    // printf("gyro data %f %f %f \n",gyro[0], gyro[1], gyro[2]);
#endif
}

static void _read_from_flash(void)
{
    union embedi_imu {
        uint8_t _buf[12];
        struct cali_data {
            struct imu_bias accel;
            struct imu_bias gyro;
        } bias;
    };

    union embedi_imu read_data;
    uint8_t len = sizeof(read_data._buf) / 2 + ((sizeof(read_data._buf) % 2) ? 1 : 0);

    embedi_read_flash(IMU_ADDR, (uint16_t *)read_data._buf, len);
    printf("read %d %d %d %d %d %d len: %d\n",
           read_data.bias.accel.x_bias,
           read_data.bias.accel.y_bias,
           read_data.bias.accel.z_bias,
           read_data.bias.gyro.x_bias,
           read_data.bias.gyro.y_bias,
           read_data.bias.gyro.z_bias, len);

    accel_bias.x_bias = read_data.bias.accel.x_bias;
    accel_bias.y_bias = read_data.bias.accel.y_bias;
    accel_bias.z_bias = read_data.bias.accel.z_bias;
    gyro_bias.x_bias = read_data.bias.gyro.x_bias;
    gyro_bias.y_bias = read_data.bias.gyro.y_bias;
    gyro_bias.z_bias = read_data.bias.gyro.z_bias;
}

static void _write_to_flash(void)
{
    union embedi_imu {
        uint8_t _buf[12];
        struct cali_data {
            struct imu_bias accel;
            struct imu_bias gyro;
        } bias;
    };

    union embedi_imu write_data;
    union embedi_imu read_data;

    write_data.bias.accel.x_bias = accel_bias.x_bias;
    write_data.bias.accel.y_bias = accel_bias.y_bias;
    write_data.bias.accel.z_bias = accel_bias.z_bias;
    write_data.bias.gyro.x_bias = gyro_bias.x_bias;
    write_data.bias.gyro.y_bias = gyro_bias.y_bias;
    write_data.bias.gyro.z_bias = gyro_bias.z_bias;

    uint8_t len = sizeof(write_data._buf) / 2 + ((sizeof(write_data._buf) % 2) ? 1 : 0);

    if (run_test == IMU_FLASH_WRITE) {
        printf("write %d %d %d %d %d %d len: %d\n",
               write_data.bias.accel.x_bias,
               write_data.bias.accel.y_bias,
               write_data.bias.accel.z_bias,
               write_data.bias.gyro.x_bias,
               write_data.bias.gyro.y_bias,
               write_data.bias.gyro.z_bias, len);
        embedi_write_flash(IMU_ADDR, (uint16_t *)write_data._buf, len);
    } else if (run_test == IMU_FLASH_READ) {
        embedi_read_flash(IMU_ADDR, (uint16_t *)read_data._buf, len);
        printf("read %d %d %d %d %d %d len: %d\n",
               read_data.bias.accel.x_bias,
               read_data.bias.accel.y_bias,
               read_data.bias.accel.z_bias,
               read_data.bias.gyro.x_bias,
               read_data.bias.gyro.y_bias,
               read_data.bias.gyro.z_bias, len);
    }
}

void embedi_imu_calibration(void)
{
    if (run_test == IMU_CALIBRATION) { // 3 scall
        _accel_callibratin();
        _gyro_callibratin();
    }
    _write_to_flash();
}

void embedi_get_accel_data(float *accel_data)
{
    short data_short[3];
    long data[3];

    mpu_get_accel_reg(data_short, NULL);
    data[0] = (long)data_short[0];
    data[1] = (long)data_short[1];
    data[2] = (long)data_short[2];
    _accel_data_collection(data);

    data[0] += accel_bias.x_bias;
    data[1] += accel_bias.y_bias;
    data[2] += accel_bias.z_bias;
    _accel_data_standardize(data, accel_data);
}

void embedi_get_gyro_data(float *gyro_data)
{
    short data_short[3];
    long data[3];

    mpu_get_gyro_reg(data_short, NULL);
    data[0] = (long)data_short[0];
    data[1] = (long)data_short[1];
    data[2] = (long)data_short[2];
    _gyro_data_collection(data);

    data[0] += gyro_bias.x_bias;
    data[1] += gyro_bias.y_bias;
    data[2] += gyro_bias.z_bias;
    _gyro_data_standardize(data, gyro_data);
}

void embedi_get_roll_angle(float *angle)
{
    struct matrix *m;
    float accel[3];
    float gyro[3];
    float accel_angle = 0;

    embedi_get_accel_data(accel);
    embedi_get_gyro_data(gyro);
    accel_angle = embedi_arctan(accel[1] / accel[2]);

    embedi_kalman_filter(accel_angle, gyro[0]);
    m = emebedi_get_kalman_estimation();
    *angle = m->matrix[0][0] * ANGLE_DIRECTION;

#ifdef CFG_IMU_DATA_SCOPE_SHOW
#if (ROLL_ANGLE_SHOW == 1)
    embedi_data_to_scope(accel_angle * 1000, CHANNEL_1);
    embedi_data_to_scope(m->matrix[0][0] * 1000, CHANNEL_2);
    embedi_data_to_scope(m->matrix[1][0] * 1000, CHANNEL_3);
    embedi_scope_show();
#endif
#else
#if 0
    printf("angle: %d %d %d\n", (int)(accel_angle * 1000 * ANGLE_DIRECTION),
           (int)(m->matrix[0][0] * 1000 * ANGLE_DIRECTION),
           (int)(m->matrix[1][0] * 1000 * ANGLE_DIRECTION));
#endif
#endif
}
