#include "embedi_config.h"
#include "embedi_test.h"
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#endif
#include <stdio.h>

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

int myabs(int a)
{
    int temp;
    if (a < 0)
        temp = -a;
    else
        temp = a;
    return temp;
}

void Set_Pwm(int motor_left, int motor_right)
{
    PWMB = myabs(motor_left);
    PWMA = myabs(motor_right);
}

/*
Fpwm(HZ) = corefrequncy / ((ARR+1)*(PSC+1))
duty(%) =  = TIM3->CCR2 / ARR(Auto Reload Register)
*/
extern int run_test;
void motor_test(void)
{
    if (run_test == MOTOR_START_BACKWARD) { // 2 scall
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        Set_Pwm(2000, 2000);
#ifndef CFG_IMU_DATA_SCOPE_SHOW
        printf("motor running...\n");
#endif
    } else if (run_test == MOTOR_START_FORDWARD) { // 1 scall
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        Set_Pwm(2000, 2000);
#ifndef CFG_IMU_DATA_SCOPE_SHOW
        printf("motor running...\n");
#endif
    } else if (run_test == MOTOR_STOP) { // 0 scall
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }

#ifndef CFG_IMU_DATA_SCOPE_SHOW
    int data = 0;

    data = _read_encoder(2);
    printf("L data %d \n", data);

    data = _read_encoder(4);
    printf("R data %d \n", data);
#endif
}

