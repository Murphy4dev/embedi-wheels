#include "cmsis_os.h"
#include "embedi_delay.h"
#include "embedi_flash.h"
#include "embedi_i2c.h"
#include "embedi_imu.h"
#include "embedi_kalman.h"
#include "embedi_motor.h"
#include "embedi_test.h"
#include "main.h"
#include <stdio.h>
#ifdef CFG_UART_ENABLE
#include "embedi_uart.h"
#endif

#define TEST_MODE 0
osThreadId _task_handle;
void embedi_task(void const *argument);
SemaphoreHandle_t xSemaphore = NULL;

void embedi_task_function(void const *argument)
{
    extern int run_test;
    float accel_data[3];
    float gyro_data[3];
    int r_speed = 0;
    int l_speed = 0;

    // need to IMU systick
    embedi_imu_enable();
    for (;;) {
#if TEST_MODE
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        osDelay(200);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        osDelay(200);
        motor_test();
        emebedi_i2c_test();
        embedi_imu_calibration();
        embedi_flash_test();
        run_test = 0;
#else
        if (xSemaphoreTake(xSemaphore, 0) == pdTRUE) {
            if (run_test == RUNNING_SWICH) {
                embedi_set_direction(FORDWARD);
                embedi_motor_start(5000, 5000);
                embedi_get_accel_data(accel_data);
                embedi_get_gyro_data(gyro_data);
                embedi_kalman_filter(accel_data[1], accel_data[2], gyro_data[0]);
                embedi_get_speed(&r_speed, &l_speed);
            } else {
                embedi_motor_sotp();
            }
        }
#endif
    }
}

void embedi_init(void)
{
    extern TIM_HandleTypeDef htim1;
    extern TIM_HandleTypeDef htim2;
    extern TIM_HandleTypeDef htim4;

    xSemaphore = xSemaphoreCreateBinary();
    osThreadDef(embedi_task, embedi_task_function, osPriorityNormal, 0, 512);
    _task_handle = osThreadCreate(osThread(embedi_task), NULL);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
#ifdef CFG_UART_ENABLE
    embedi_enable_uart1_interrupt();
#endif
    embedi_imu_init();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12) {
        xSemaphoreGiveFromISR(xSemaphore, NULL);
    }
}
