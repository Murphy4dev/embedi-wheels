#include "cmsis_os.h"
#include "embedi_delay.h"
#include "embedi_flash.h"
#include "embedi_i2c.h"
#include "embedi_imu.h"
#include "embedi_kalman.h"
#include "embedi_motor.h"
#include "embedi_pid.h"
#include "embedi_scope.h"
#include "embedi_test.h"
#include "main.h"
#include <stdio.h>
#ifdef CFG_UART_ENABLE
#include "embedi_uart.h"
#endif

// #define TEST_MODE
/* banlance*/
#define BALANCE_P (10000)
#define BALANCE_D (500)
#define BALANCE_T (0)
/* banlance*/
#define VELOCITY_P (100)
#define VELOCITY_I (30)
#define VELOCITY_T (0)
osThreadId _task_handle;
void embedi_task(void const *argument);
SemaphoreHandle_t xSemaphore = NULL;

static void _wheels_control(void)
{
    int r_speed = 0;
    int l_speed = 0;
    float angle = 0.0;
    float pwm = 0;
    struct _pid balance_pd;
    float balance_pwm = 0;
    struct _pid velocity_pi;
    float velocity_pwm = 0;
    embedi_pid_init(&balance_pd, BALANCE_T, BALANCE_P, 0, BALANCE_D);
    embedi_pid_init(&velocity_pi, VELOCITY_T, VELOCITY_P, VELOCITY_I, 0);
    embedi_get_roll_angle(&angle);
    embedi_get_speed(&r_speed, &l_speed);

    balance_pwm = embedi_pid(&balance_pd, angle);
    velocity_pwm += embedi_delta_pid(&velocity_pi, (r_speed + l_speed) / 2);
    pwm = balance_pwm + velocity_pwm;
#ifdef CFG_IMU_DATA_SCOPE_SHOW
#if (VELOCITY_DATA_SHOW == 1)
    // embedi_data_to_scope(pwm, CHANNEL_1);
    // embedi_data_to_scope(balance_pwm, CHANNEL_2);
    // embedi_data_to_scope(velocity_pwm, CHANNEL_3);
    // embedi_data_to_scope((r_speed + l_speed) / 2, CHANNEL_3);
    // embedi_scope_show();
    printf("%d %d %d \n", (int)pwm, (int)balance_pwm, (int)velocity_pwm);
#endif
#endif

    if (pwm < 0) {
        embedi_set_direction(FORDWARD);
    } else {
        embedi_set_direction(BACKWARD);
    }
    embedi_motor_start((int)pwm, (int)pwm);
}

void embedi_task_function(void const *argument)
{
    extern int run_test;
    // need to IMU systick
    embedi_imu_enable();
    for (;;) {
#ifdef TEST_MODE
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
            if (run_test == MOTOR_START_FORDWARD) {
                _wheels_control();
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
