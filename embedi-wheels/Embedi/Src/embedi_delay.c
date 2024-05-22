#include "embedi_config.h"
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
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

void embedi_delay_us(uint32_t microsec)
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

void embedi_delay_ms(uint16_t millisec)
{
    uint16_t i = 0;

    for (i = 0; i<millisec; i++) 
        embedi_delay_us(1000);
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
