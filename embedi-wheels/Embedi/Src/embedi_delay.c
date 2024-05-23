#include "embedi_config.h"
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#endif

#ifdef CFG_TIMER_DELAY_ENABLE
#if (CFG_DELAY_TIMER_INDEX == 1)
extern TIM_HandleTypeDef htim1;
#elif (CFG_DELAY_TIMER_INDEX == 2)
extern TIM_HandleTypeDef htim2;
#elif (CFG_DELAY_TIMER_INDEX == 3)
extern TIM_HandleTypeDef htim3;
#elif (CFG_DELAY_TIMER_INDEX == 4)
extern TIM_HandleTypeDef htim4;
#elif (CFG_DELAY_TIMER_INDEX == 5)
extern TIM_HandleTypeDef htim5;
#elif (CFG_DELAY_TIMER_INDEX == 6)
extern TIM_HandleTypeDef htim6;
#elif (CFG_DELAY_TIMER_INDEX == 7)
extern TIM_HandleTypeDef htim7;
#endif

static void __embedi_delay_us(uint32_t microsec)
{
    TIM_HandleTypeDef *timer = NULL;

    uint16_t  differ = 0xffff - microsec - 5;
#if (CFG_DELAY_TIMER_INDEX == 1)
    timer = &htim1;
#elif (CFG_DELAY_TIMER_INDEX == 2)
    timer = &htim2;
#elif (CFG_DELAY_TIMER_INDEX == 3)
    timer = &htim3;
#elif (CFG_DELAY_TIMER_INDEX == 4)
    timer = &htim4;
#elif (CFG_DELAY_TIMER_INDEX == 5)
    timer = &htim5;
#elif (CFG_DELAY_TIMER_INDEX == 6)
    timer = &htim6;
#elif (CFG_DELAY_TIMER_INDEX == 7)
    timer = &htim7;
#endif
    __HAL_TIM_SetCounter(timer, differ);
    HAL_TIM_Base_Start(timer);

    while (differ < 0xffff-5) {
        differ = __HAL_TIM_GetCounter(timer);
    };

    HAL_TIM_Base_Stop(timer);
}

void embedi_delay_us(uint32_t microsec)
{
    __embedi_delay_us(microsec);
}

void embedi_delay_ms(uint16_t millisec)
{
    uint16_t i = 0;

    for (i = 0; i < millisec; i++) {
        __embedi_delay_us(1000);
    }

}
#else
__asm void _delay_loop(uint32_t count)
{
    /* 1 instruction sycle */
    subs    r0, #1; 
    /* 3 instruction sycle */
    bne     _delay_loop;
    /* just run once ignore*/
    bx      lr;
}

void embedi_delay_us(uint32_t microsec)
{
    #define LOOP_CYCLE 4
    uint32_t count = microsec * CFG_SYSTEM_CLOCK /  LOOP_CYCLE ;

    _delay_loop(count);
}

void embedi_delay_ms(uint16_t millisec)
{
    embedi_delay_us(millisec * 1000);
}

#endif /*CFG_EMBEDI_DELAY_ENABLE*/
