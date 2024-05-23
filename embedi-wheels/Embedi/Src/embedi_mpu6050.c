#include "embedi_config.h"
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#endif
#include "embedi_i2c.h"
#include "embedi_mpu6050.h"
#include <stdio.h>

void embedi_mpu6050_init(void)
{
    int ret = 0;
    uint8_t data = 0;

    ret = embedi_i2c_read_byte(MPU6050_ADRR, MPU6050_RA_WHO_AM_I, &data);
    if (ret) {
        printf("ret:%d fail to init mpu6050 \r\n", ret);
        return;
    }
    if (data != 0x68) {
        printf("ret:%d fail to init mpu6050 \r\n", ret);
        return;
    } else {
        printf("succeed to get mpu6050, ID:0x%x \r\n", data);
    }

    embedi_i2c_write_byte(MPU6050_ADRR, MPU6050_RA_PWR_MGMT_1, 0x01);
    embedi_i2c_write_byte(MPU6050_ADRR, MPU6050_RA_PWR_MGMT_2, 0x09);
    embedi_i2c_write_byte(MPU6050_ADRR, MPU6050_RA_SMPLRT_DIV, 0x06);
    embedi_i2c_write_byte(MPU6050_ADRR, MPU6050_RA_CONFIG, 0x18);
    /* ±2000°/s */
    embedi_i2c_write_byte(MPU6050_ADRR, MPU6050_RA_GYRO_CONFIG, 0x18);
    /* ±16g */
    embedi_i2c_write_byte(MPU6050_ADRR, MPU6050_RA_ACCEL_CONFIG, 0x18);
}
