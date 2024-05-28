#include "embedi_motor.h"
#include "embedi_config.h"
#include "embedi_test.h"
#include <stdio.h>
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#endif

#define SPEED_DIRECTION (-1)     // 0 or -1
#define RIGHT_LEFT_DIRECTION (1) // 0 or 1
#define MAX_DUTY 7199
extern int run_test;

static int _read_encoder(int tim)
{
    int Encoder_TIM;
    switch (tim) {
    case 2:
        Encoder_TIM = (short)TIM2->CNT;
        TIM2->CNT = 0;
        break;
    case 3:
        Encoder_TIM = (short)TIM3->CNT;
        TIM3->CNT = 0;
        break;
    case 4:
        Encoder_TIM = (short)TIM4->CNT;
        TIM4->CNT = 0;
        break;
    default:
        Encoder_TIM = 0;
    }
    return Encoder_TIM;
}

#define PWMB TIM1->CCR4 // PA11
#define PWMA TIM1->CCR1 // PA8

static int _abs(int a)
{
    int temp;
    if (a < 0)
        temp = -a;
    else
        temp = a;
    return temp;
}

static void _set_pwm(int motor_left, int motor_right)
{
    PWMB = _abs(motor_left);
    PWMA = _abs(motor_right);
}

void embedi_get_speed(int *right, int *left)
{
    *right = _read_encoder(2) * SPEED_DIRECTION;
#ifndef CFG_IMU_DATA_SCOPE_SHOW
    // printf("L data: %d \n", *right);
#endif

    *left = _read_encoder(4) * SPEED_DIRECTION;
#ifndef CFG_IMU_DATA_SCOPE_SHOW
    // printf("R data: %d \n", *left);
#endif
}

void embedi_set_direction(enum embedi_direction dir)
{
    if (dir == FORDWARD) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

void embedi_motor_start(int left, int right)
{
    if (left > MAX_DUTY) {
        left = MAX_DUTY;
    }
    if (right > MAX_DUTY) {
        right = MAX_DUTY;
    }
    if (RIGHT_LEFT_DIRECTION) {
        _set_pwm(left, right);
    } else {
        _set_pwm(right, left);
    }
}

void embedi_motor_sotp(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    _set_pwm(0, 0);
}

/*
Fpwm(HZ) = corefrequncy / ((ARR+1)*(PSC+1))
duty(%) =  = TIM3->CCR2 / ARR(Auto Reload Register)
*/
void motor_test(void)
{
    embedi_set_direction(FORDWARD);
    embedi_motor_start(2000, 2000);
}
