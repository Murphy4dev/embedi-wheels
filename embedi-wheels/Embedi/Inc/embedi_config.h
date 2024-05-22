#ifndef __EMBEDI_CONFIG_H
#define __EMBEDI_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
#define CFG_STM32F1XX 1

#define CFG_UART_ENABLE 1
#define CFG_PRINTF_TO_UART 1

#define CFG_EMBEDI_DELAY_ENABLE 1
#define CFG_DELAY_USE_TIMER 1
#define CFG_DELAY_TIMER 2

#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_CONFIG_H */
