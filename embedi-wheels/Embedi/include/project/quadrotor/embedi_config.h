#ifndef __EMBEDI_CONFIG_H
#define __EMBEDI_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
#define CFG_STM32F1XX

#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#elif defined CFG_STM32F4XX
#include "stm32f4xx_hal.h"
#else
#error no specific platfom
#endif

#define CFG_FREERTOS_ENABLE

/* UART CONFIG*/
#define CFG_UART_ENABLE
#define CFG_PRINTF_TO_UART_INDEX 1

/* DELAY CONFIG */
//#define CFG_TIMER_DELAY_ENABLE
//#define CFG_DELAY_TIMER_INDEX 2
#define CFG_SYSTEM_CLOCK 72 //nop == 1 / CFG_SYSTEM_CLOCK us

/* I2C CONFIG */
#define CFG_I2C_GPIO_SIMULATION
#define CFG_I2C_INDEX 1

/* PWM CONFIG */
#define CFG_MOTOR_ENBALE
#define CFG_PWM_TIMER htim1
#define CFG_ENCODER1_TIMER htim2
#define CFG_ENCODER2_TIMER htim4

/* IMU data show just only one micro is 1*/
#define CFG_IMU_DATA_SCOPE_SHOW
#define ACCEL_DATA_SHOW 0
#define GRYO_DATA_SHOW 0
#define ROLL_ANGLE_SHOW 0
#define EULER_ANGLE_SHOW 0
#define VELOCITY_DATA_SHOW 1

#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_CONFIG_H */
