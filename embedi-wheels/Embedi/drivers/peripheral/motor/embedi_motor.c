#include "embedi_motor.h"
#include "embedi_config.h"
#include "embedi_module_init.h"
#include "embedi_test.h"
#include <stdio.h>

#ifndef CFG_MOTOR_ENBALE
#warning you need to do specific motor configration and enable CFG_MOTOR_ENBALE in embedi_config.h
#endif

#define SPEED_DIRECTION (-1)     // 0 or -1
#define RIGHT_LEFT_DIRECTION (1) // 0 or 1
#define MAX_DUTY 7199

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
    int force_stop = 0;

    if (left > MAX_DUTY) {
        left = MAX_DUTY;
        force_stop = 1;
    } else if (left < -MAX_DUTY) {
        left = -MAX_DUTY;
        force_stop = 1;
    }

    if (right > MAX_DUTY) {
        right = MAX_DUTY;
        force_stop = 1;
    } else if (right < -MAX_DUTY) {
        right = -MAX_DUTY;
        force_stop = 1;
    }
    force_stop = 0;
    if (force_stop) {
        _set_pwm(0, 0);
        return;
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

static void motor_init(void)
{
#ifdef CFG_MOTOR_ENBALE
    extern TIM_HandleTypeDef CFG_PWM_TIMER;
    extern TIM_HandleTypeDef CFG_ENCODER1_TIMER;
    extern TIM_HandleTypeDef CFG_ENCODER2_TIMER;

    HAL_TIM_PWM_Start(&CFG_PWM_TIMER, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&CFG_PWM_TIMER, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&CFG_ENCODER1_TIMER, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&CFG_ENCODER2_TIMER, TIM_CHANNEL_ALL);
#endif
}

driver_init(motor_init);
