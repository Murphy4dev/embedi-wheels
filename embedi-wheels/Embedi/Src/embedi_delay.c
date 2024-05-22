#include "embedi_config.h"
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#endif
#if (CFG_EMBEDI_DELAY_ENABLE == 1)

#if (CFG_DELAY_TIMER == 1)
extern TIM_HandleTypeDef htim1;
#elif (CFG_DELAY_TIMER == 2)
extern TIM_HandleTypeDef htim2;
#elif (CFG_DELAY_TIMER == 3)
extern TIM_HandleTypeDef htim3;
#elif (CFG_DELAY_TIMER == 4)
extern TIM_HandleTypeDef htim4;
#elif (CFG_DELAY_TIMER == 5)
extern TIM_HandleTypeDef htim5;
#elif (CFG_DELAY_TIMER == 6)
extern TIM_HandleTypeDef htim6;
#elif (CFG_DELAY_TIMER == 7)
extern TIM_HandleTypeDef htim7;
#endif

static uint8_t timeout = 0;

static void __embedi_delay_us(uint32_t microsec)
{
    TIM_HandleTypeDef *timer = NULL;

    uint16_t  differ = 0xffff - microsec - 5;
#if (CFG_DELAY_TIMER == 1)
    timer = &htim1;
#elif (CFG_DELAY_TIMER == 2)
    timer = &htim2;
#elif (CFG_DELAY_TIMER == 3)
    timer = &htim3;
#elif (CFG_DELAY_TIMER == 4)
    timer = &htim4;
#elif (CFG_DELAY_TIMER == 5)
    timer = &htim5;
#elif (CFG_DELAY_TIMER == 6)
    timer = &htim6;
#elif (CFG_DELAY_TIMER == 7)
    timer = &htim7;
#endif
    __HAL_TIM_SetCounter(timer, differ);
    HAL_TIM_Base_Start(timer);

    while (differ < 0xffff-5) {
        differ = __HAL_TIM_GetCounter(timer);
    };

    HAL_TIM_Base_Stop(timer);
}

static void __embedi_it_delay_us(uint32_t microsec)
{
    TIM_HandleTypeDef *timer = NULL;
#if (CFG_DELAY_TIMER == 1)
    timer = &htim1;
#elif (CFG_DELAY_TIMER == 2)
    timer = &htim2;
#elif (CFG_DELAY_TIMER == 3)
    timer = &htim3;
#elif (CFG_DELAY_TIMER == 4)
    timer = &htim4;
#elif (CFG_DELAY_TIMER == 5)
    timer = &htim5;
#elif (CFG_DELAY_TIMER == 6)
    timer = &htim6;
#elif (CFG_DELAY_TIMER == 7)
    timer = &htim7;
#endif
    if (timer) {
        timeout = 0;
        timer->Init.Period = microsec - 1;
        HAL_TIM_Base_Init(timer);
        HAL_TIM_Base_Start_IT(timer);

        do {
            __NOP();
        } while (timeout == 0);
        HAL_TIM_Base_Stop_IT(timer);
    }
}

void embedi_delay_us(uint32_t microsec)
{
#if (CFG_DELAY_USE_TIMER_IT == 1)
    __embedi_it_delay_us(microsec);
#else
    __embedi_delay_us(microsec);
#endif
}

void embedi_delay_ms(uint16_t millisec)
{
    uint16_t i = 0;

    for (i = 0; i < millisec; i++) {
    #if (CFG_DELAY_USE_TIMER_IT == 1)
        __embedi_it_delay_us(1000);
    #else
        __embedi_delay_us(1000);
    #endif
    }

}

void embedi_delay_timer_it_callback(TIM_HandleTypeDef *htim)
{
    TIM_HandleTypeDef *timer = NULL;
#if (CFG_DELAY_TIMER == 1)
    timer = &htim1;
#elif (CFG_DELAY_TIMER == 2)
    timer = &htim2;
#elif (CFG_DELAY_TIMER == 3)
    timer = &htim3;
#elif (CFG_DELAY_TIMER == 4)
    timer = &htim4;
#elif (CFG_DELAY_TIMER == 5)
    timer = &htim5;
#elif (CFG_DELAY_TIMER == 6)
    timer = &htim6;
#elif (CFG_DELAY_TIMER == 7)
    timer = &htim7;
#endif
    if (timer) {
        timeout = 1;
    }
}
#else
void embedi_delay_us(uint32_t microsec)
{
    (void)(microsec);
    __NOP();
}

void embedi_delay_ms(uint16_t millisec)
{
    (void)(millisec);
    __NOP();
}

#endif /*CFG_EMBEDI_DELAY_ENABLE*/
