#ifndef __EMBEDI_UART_H
#define __EMBEDI_UART_H

#ifdef __cplusplus
extern "C" {
#endif

void embedi_enable_uart1_interrupt(void);
void embedi_uart_send_byte(const uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_UART_H */
