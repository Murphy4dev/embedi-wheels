#ifndef __EMBEDI_CONFIG_H
#define __EMBEDI_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
#define CFG_STM32F1XX

/* UART CONFIG*/
#define CFG_UART_ENABLE
#define CFG_PRINTF_TO_UART 1

/* DELAY CONFIG */
//#define CFG_TIMER_DELAY_ENABLE
#define CFG_DELAY_TIMER_INDEX 2
/* nop == 1 / CFG_SYSTEM_CLOCK us */
#define CFG_SYSTEM_CLOCK 72

#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_CONFIG_H */
