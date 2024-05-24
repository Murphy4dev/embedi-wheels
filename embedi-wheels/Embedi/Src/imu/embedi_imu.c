#include "embedi_config.h"
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#endif
#include "embedi_i2c.h"
#include <stdio.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

void embedi_mpu6050_init(void)
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
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12) {
        short data_short[3];
        long data[3];
        float accel[3];
        float gyro[3];
        unsigned short accel_sens;
    	float gyro_sens;

        mpu_get_accel_reg(data_short, NULL);
        data[0] = (long)data_short[0];
        data[1] = (long)data_short[1];
        data[2] = (long)data_short[2];
		mpu_get_accel_sens(&accel_sens);
		accel[0] = (float)data[0] / accel_sens * 9.8;
		accel[1] = (float)data[1] / accel_sens * 9.8;
		accel[2] = (float)data[2] / accel_sens * 9.8;

        //printf("accel data %f %f %f \n",accel[0], accel[1], accel[2]);

        mpu_get_gyro_reg(data_short, NULL);
        data[0] = (long)data_short[0];
        data[1] = (long)data_short[1];
        data[2] = (long)data_short[2];
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = data[0] / gyro_sens * ((2*3.1415926)/360);
		gyro[1] = data[1] / gyro_sens * ((2*3.1415926)/360);
		gyro[2] = data[2] / gyro_sens * ((2*3.1415926)/360);

        //printf("gyro data %f %f %f \n",gyro[0], gyro[1], gyro[2]);
    }
}
