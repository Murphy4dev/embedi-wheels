#include "embedi_imu.h"
#include "embedi_2d_kalman.h"
#include "embedi_6d_kalman.h"
#include "embedi_config.h"
#include "embedi_flash.h"
#include "embedi_i2c.h"
#include "embedi_math.h"
#include "embedi_module_init.h"
#include "embedi_scope.h"
#include "embedi_system.h"
#include "embedi_test.h"
#include <math.h>
#include <stdio.h>

static void _read_from_flash(void);
static struct imu_chip g_chip;

void embedi_register_operations(struct imu_operations *ops)
{
    if (!ops) {
        printf("imu ops null\n");
        return;
    }

    if (!g_chip.ops) {
        g_chip.ops = ops;
    } else {
        printf("imu ops has been register\n");
    }
}

int embedi_imu_hardware_init(struct int_param_s *int_param)
{
    if (g_chip.ops && g_chip.ops->init) {
        return g_chip.ops->init(int_param);
    }

    return -1;
}

int embedi_enable_sensor(unsigned char sensors)
{
    if (g_chip.ops && g_chip.ops->enable_sensor) {
        return g_chip.ops->enable_sensor(sensors);
    }

    return -1;
}

int embedi_configure_fifo(unsigned char sensors)
{
    if (g_chip.ops && g_chip.ops->configure_fifo) {
        return g_chip.ops->configure_fifo(sensors);
    }

    return -1;
}

int embedi_set_sample_rate(unsigned short rate)
{
    if (g_chip.ops && g_chip.ops->set_sample_rate) {
        return g_chip.ops->set_sample_rate(rate);
    }

    return -1;
}

int embedi_get_sample_rate(unsigned short *rate)
{
    if (g_chip.ops && g_chip.ops->get_sample_rate) {
        return g_chip.ops->get_sample_rate(rate);
    }

    return -1;
}

int embedi_get_gyro_fsr(unsigned short *fsr)
{
    if (g_chip.ops && g_chip.ops->get_gyro_fsr) {
        return g_chip.ops->get_gyro_fsr(fsr);
    }

    return -1;
}

int embedi_get_accel_fsr(unsigned char *fsr)
{
    if (g_chip.ops && g_chip.ops->get_accel_fsr) {
        return g_chip.ops->get_accel_fsr(fsr);
    }

    return -1;
}

int embedi_read_gyro_data(short *data, unsigned long *timestamp)
{
    if (g_chip.ops && g_chip.ops->read_gyro_data) {
        return g_chip.ops->read_gyro_data(data, timestamp);
    }

    return -1;
}

int embedi_read_accel_data(short *data, unsigned long *timestamp)
{
    if (g_chip.ops && g_chip.ops->read_accel_data) {
        return g_chip.ops->read_accel_data(data, timestamp);
    }

    return -1;
}

int embedi_get_gyro_sens(float *sens)
{
    if (g_chip.ops && g_chip.ops->get_gyro_sens) {
        return g_chip.ops->get_gyro_sens(sens);
    }

    return -1;
}

int embedi_get_accel_sens(unsigned short *sens)
{
    if (g_chip.ops && g_chip.ops->get_accel_sens) {
        return g_chip.ops->get_accel_sens(sens);
    }

    return -1;
}

void embedi_imu_init(void)
{
    struct int_param_s int_param;
    unsigned short gyro_rate, gyro_fsr;
    unsigned char accel_fsr;
    int result = 0;

    result = embedi_imu_hardware_init(&int_param);
    if (result) {
        printf("Could not initialize imu.\n");
    }
    /* Wake up all sensors. */
    embedi_enable_sensor(EMBEDI_XYZ_GYRO | EMBEDI_XYZ_ACCEL);

    /* Push both gyro and accel data into the FIFO. */
    embedi_configure_fifo(EMBEDI_XYZ_GYRO | EMBEDI_XYZ_ACCEL);
    embedi_set_sample_rate(DEFAULT_MPU_HZ);

    /* Read back configuration in case it was set improperly. */
    embedi_get_sample_rate(&gyro_rate);
    embedi_get_gyro_fsr(&gyro_fsr);
    embedi_get_accel_fsr(&accel_fsr);

    printf("imu config %d %d %d \n", gyro_rate, gyro_fsr, accel_fsr);
    _read_from_flash();

    embedi_enable_sensor(0);
    embedi_2d_kalman_init();
    embedi_6d_kalman_init();
}

void embedi_imu_enable(void)
{
    embedi_enable_sensor(EMBEDI_XYZ_GYRO | EMBEDI_XYZ_ACCEL);
}

static void _accel_data_collection(long *accel)
{
    static int index = 0;

    g_chip.accel_data[index % CALI_DATA_LEN].data[0] = accel[0];
    g_chip.accel_data[index % CALI_DATA_LEN].data[1] = accel[1];
    g_chip.accel_data[index % CALI_DATA_LEN].data[2] = accel[2];
    index++;

    if (index == CALI_DATA_LEN) {
        index = 0;
        g_chip.accel_cali_ready = 1;
    }
}

static void _gyro_data_collection(long *gyro)
{
    static int index = 0;

    g_chip.gyro_data[index % CALI_DATA_LEN].data[0] = gyro[0];
    g_chip.gyro_data[index % CALI_DATA_LEN].data[1] = gyro[1];
    g_chip.gyro_data[index % CALI_DATA_LEN].data[2] = gyro[2];
    index++;

    if (index == CALI_DATA_LEN) {
        index = 0;
        g_chip.gyro_cali_ready = 1;
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

    if (!g_chip.accel_cali_ready) {
        printf("accel cali collection not ready \n");
        return;
    }
    x_sum = y_sum = z_sum = 0;
    x_avg = y_avg = z_avg = 0;

    for (index = 0; index < CALI_DATA_LEN; index++) {
        x_sum += g_chip.accel_data[index].data[0];
        y_sum += g_chip.accel_data[index].data[1];
        z_sum += g_chip.accel_data[index].data[2];
        printf("acc cali:[%d, %d, %d]\n",
               g_chip.accel_data[index].data[0], g_chip.accel_data[index].data[1],
               g_chip.accel_data[index].data[2]);
    }

    x_avg = x_sum / CALI_DATA_LEN;
    y_avg = y_sum / CALI_DATA_LEN;
    z_avg = z_sum / CALI_DATA_LEN;
    printf("acc avg:[%d, %d, %d]\n", x_avg, y_avg, z_avg);

    embedi_get_accel_sens(&accel_sens);
    g_chip.accel_bias.x_bias = (ACCEL_X_TARGET * accel_sens / _G - x_avg);
    g_chip.accel_bias.y_bias = (ACCEL_Y_TARGET * accel_sens / _G - y_avg);
    g_chip.accel_bias.z_bias = (ACCEL_Z_TARGET * accel_sens / _G - z_avg);
    printf("acc z_target_raw:[%d] accel_sens:[%d]\n",
           ACCEL_Z_TARGET * accel_sens / _G, accel_sens);
    printf("acc bias:[%d, %d, %d]\n",
           g_chip.accel_bias.x_bias, g_chip.accel_bias.y_bias,
           g_chip.accel_bias.z_bias);
}

static void _gyro_callibratin(void)
{
    int index = 0;
    int x_sum, y_sum, z_sum;
    int x_avg, y_avg, z_avg;

    if (!g_chip.gyro_cali_ready) {
        printf("gyro cali collection not ready \n");
        return;
    }

    x_sum = y_sum = z_sum = 0;
    x_avg = y_avg = z_avg = 0;

    for (index = 0; index < CALI_DATA_LEN; index++) {
        x_sum += g_chip.gyro_data[index].data[0];
        y_sum += g_chip.gyro_data[index].data[1];
        z_sum += g_chip.gyro_data[index].data[2];
        printf("gyro cali:[%d, %d, %d]\n",
               g_chip.gyro_data[index].data[0], g_chip.gyro_data[index].data[1],
               g_chip.gyro_data[index].data[2]);
    }

    x_avg = x_sum / CALI_DATA_LEN;
    y_avg = y_sum / CALI_DATA_LEN;
    z_avg = z_sum / CALI_DATA_LEN;
    printf("gyro avg:[%d, %d, %d]\n", x_avg, y_avg, z_avg);

    g_chip.gyro_bias.x_bias = GYRO_TARGET - x_avg;
    g_chip.gyro_bias.y_bias = GYRO_TARGET - y_avg;
    g_chip.gyro_bias.z_bias = GYRO_TARGET - z_avg;

    printf("gyro bias:[%d, %d, %d]\n",
           g_chip.gyro_bias.x_bias, g_chip.gyro_bias.y_bias,
           g_chip.gyro_bias.z_bias);
}

static void _accel_data_standardize(long *data, float *accel)
{
    unsigned short accel_sens;

    if (!accel || !data)
        return;
    embedi_get_accel_sens(&accel_sens);
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
    printf("accel data %f %f %f \n", accel[0], accel[1], accel[2]);
#endif
}

static void _gyro_data_standardize(long *data, float *gyro)
{
    float gyro_sens;

    if (!gyro || !data)
        return;

    embedi_get_gyro_sens(&gyro_sens);
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

    //embedi_read_flash(IMU_ADDR, (uint16_t *)read_data._buf, len);
    read_data.bias.accel.x_bias = -905;
    read_data.bias.accel.y_bias = -339;
    read_data.bias.accel.z_bias = 2168;
    read_data.bias.gyro.x_bias = 4;
    read_data.bias.gyro.y_bias = 2;
    read_data.bias.gyro.z_bias = -26;
    printf("read %d %d %d %d %d %d len: %d\n",
           read_data.bias.accel.x_bias,
           read_data.bias.accel.y_bias,
           read_data.bias.accel.z_bias,
           read_data.bias.gyro.x_bias,
           read_data.bias.gyro.y_bias,
           read_data.bias.gyro.z_bias, len);

    g_chip.accel_bias.x_bias = read_data.bias.accel.x_bias;
    g_chip.accel_bias.y_bias = read_data.bias.accel.y_bias;
    g_chip.accel_bias.z_bias = read_data.bias.accel.z_bias;
    g_chip.gyro_bias.x_bias = read_data.bias.gyro.x_bias;
    g_chip.gyro_bias.y_bias = read_data.bias.gyro.y_bias;
    g_chip.gyro_bias.z_bias = read_data.bias.gyro.z_bias;
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

    write_data.bias.accel.x_bias = g_chip.accel_bias.x_bias;
    write_data.bias.accel.y_bias = g_chip.accel_bias.y_bias;
    write_data.bias.accel.z_bias = g_chip.accel_bias.z_bias;
    write_data.bias.gyro.x_bias = g_chip.gyro_bias.x_bias;
    write_data.bias.gyro.y_bias = g_chip.gyro_bias.y_bias;
    write_data.bias.gyro.z_bias = g_chip.gyro_bias.z_bias;

    uint8_t len = sizeof(write_data._buf) / 2 + ((sizeof(write_data._buf) % 2) ? 1 : 0);

    printf("write %d %d %d %d %d %d len: %d\n",
           write_data.bias.accel.x_bias,
           write_data.bias.accel.y_bias,
           write_data.bias.accel.z_bias,
           write_data.bias.gyro.x_bias,
           write_data.bias.gyro.y_bias,
           write_data.bias.gyro.z_bias, len);
    embedi_write_flash(IMU_ADDR, (uint16_t *)write_data._buf, len);

    embedi_read_flash(IMU_ADDR, (uint16_t *)read_data._buf, len);
    printf("read %d %d %d %d %d %d len: %d\n",
           read_data.bias.accel.x_bias,
           read_data.bias.accel.y_bias,
           read_data.bias.accel.z_bias,
           read_data.bias.gyro.x_bias,
           read_data.bias.gyro.y_bias,
           read_data.bias.gyro.z_bias, len);
}

void embedi_imu_calibration(void)
{
    if (embedi_get_run_state() == IMU_CALIBRATION) { // 3 scall
        _accel_callibratin();
        _gyro_callibratin();
    }
    _write_to_flash();
}

void embedi_get_accel_data(float *accel_data)
{
    short data_short[3];
    long data[3];

    embedi_read_accel_data(data_short, NULL);
    data[0] = (long)data_short[0];
    data[1] = (long)data_short[1];
    data[2] = (long)data_short[2];
    _accel_data_collection(data);

    data[0] += g_chip.accel_bias.x_bias;
    data[1] += g_chip.accel_bias.y_bias;
    data[2] += g_chip.accel_bias.z_bias;
    _accel_data_standardize(data, accel_data);
}

void embedi_get_gyro_data(float *gyro_data)
{
    short data_short[3];
    long data[3];

    embedi_read_gyro_data(data_short, NULL);
    data[0] = (long)data_short[0];
    data[1] = (long)data_short[1];
    data[2] = (long)data_short[2];
    _gyro_data_collection(data);

    data[0] += g_chip.gyro_bias.x_bias;
    data[1] += g_chip.gyro_bias.y_bias;
    data[2] += g_chip.gyro_bias.z_bias;
    _gyro_data_standardize(data, gyro_data);
}

int embedi_update_imu_data_buff(void)
{
    float accel[3];
    float gyro[3];

    embedi_get_accel_data(accel);
    embedi_get_gyro_data(gyro);

    return (g_chip.gyro_cali_ready && g_chip.accel_cali_ready);
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

    embedi_2d_kalman_filter(accel_angle, gyro[0]);
    m = emebedi_get_2d_kalman_obsever();
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
    printf("angle: %f %f %f\n", (accel_angle * ANGLE_DIRECTION),
           (m->matrix[0][0] * ANGLE_DIRECTION),
           (m->matrix[1][0] * ANGLE_DIRECTION));
#endif
#endif
}
/*
    angle[0] = roll
    angle[1] = pitch
    angle[2] = yaw
*/
void embedi_get_euler_angle(float *angle)
{
    struct matrix *m;
    float accel[3];
    float gyro[3];

    if (!angle) {
        printf("angle null \n");
        return;
    }

    embedi_get_accel_data(accel);
    embedi_get_gyro_data(gyro);

    embedi_6d_kalman_filter(accel, gyro);
    m = emebedi_get_6d_kalman_obsever();
    // embedi_print_matrix(m);
    float q0 = m->matrix[0][0];
    float q1 = m->matrix[1][0];
    float q2 = m->matrix[2][0];
    float q3 = m->matrix[3][0];
    angle[0] = -atan(2 * (q2 * q3 + q0 * q1) / (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3));
    angle[1] = asin(2 * (q1 * q3 - q0 * q2));
    angle[2] = atan(2 * (q1 * q2 + q0 * q3) / (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3));

#ifdef CFG_IMU_DATA_SCOPE_SHOW
#if (EULER_ANGLE_SHOW == 1)
#if 0
    struct matrix rot;
    struct matrix fusion_accel;
    struct matrix vector;

    embedi_create_matrix(&fusion_accel, 3, 1);
    embedi_create_matrix(&vector, 3, 1);
    vector.matrix[0][0] = 0;
    vector.matrix[1][0] = 0;
    vector.matrix[2][0] = 1;
    embedi_create_matrix(&rot, 3, 3);
    embedi_quat_to_rot(&rot);
    // fusion_accel = vector([0 0 9.8]) * rot
    embedi_matrix_mul(&rot, &vector, &fusion_accel);

    embedi_data_to_scope(accel[0] * 10, CHANNEL_1);
    embedi_data_to_scope(-fusion_accel.matrix[0][0] * 10, CHANNEL_2);
    embedi_data_to_scope(accel[1] * 10, CHANNEL_3);
    embedi_data_to_scope(-fusion_accel.matrix[1][0] * 10, CHANNEL_4);
    embedi_data_to_scope(accel[2] * 10, CHANNEL_5);
    embedi_data_to_scope(fusion_accel.matrix[2][0] * 10, CHANNEL_6);
#endif
    // embedi_data_to_scope(angle[0] * rad2deg, CHANNEL_1);
    // embedi_data_to_scope(angle[1] * rad2deg, CHANNEL_2);
    // embedi_data_to_scope(angle[2] * rad2deg, CHANNEL_3);
    // embedi_scope_show();
    //  printf("quat: %f %f %f %f\n", q0, q1, q2, q3);
    printf("angle: %f %f %f\n",
           angle[0] * rad2deg,
           angle[1] * rad2deg,
           angle[2] * rad2deg);
#endif
#else
#if 0
    printf("angle: %f %f %f\n",
        angle[0] * rad2deg,
        angle[1] * rad2deg, 
        angle[2] * rad2deg);
#endif
#endif
}

driver_init(embedi_imu_init);
