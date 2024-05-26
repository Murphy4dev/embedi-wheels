#include "cmsis_os.h"
#include "embedi_delay.h"
#include "embedi_flash.h"
#include "embedi_i2c.h"
#include "embedi_imu.h"
#include "embedi_motor.h"
#include "main.h"
#include <stdio.h>
#ifdef CFG_UART_ENABLE
#include "embedi_uart.h"
#endif
osThreadId _task_handle;
void embedi_task(void const *argument);

void embedi_task_function(void const *argument)
{
    /* USER CODE BEGIN 5 */
    extern int run_test;
    /* Infinite loop */
    for (;;) {
#if 1
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
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        embedi_delay_ms(200);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        embedi_delay_ms(200);
        emebedi_i2c_test();
#endif
    }
    /* USER CODE END 5 */
}

void embedi_init(void)
{
    extern TIM_HandleTypeDef htim1;
    extern TIM_HandleTypeDef htim2;
    extern TIM_HandleTypeDef htim4;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
#ifdef CFG_UART_ENABLE
    embedi_enable_uart1_interrupt();
#endif
    embedi_imu_init();

    osThreadDef(embedi_task, embedi_task_function, osPriorityNormal, 0, 128);
    _task_handle = osThreadCreate(osThread(embedi_task), NULL);
}
