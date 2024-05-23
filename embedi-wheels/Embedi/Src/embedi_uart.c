#include <stdio.h>
#include "embedi_config.h"
#ifdef CFG_STM32F1XX
#include "stm32f1xx_hal.h"
#endif

#define UART_BUFF_LEN 1
#define UART_TIMEOUT 100

#pragma import(__use_no_semihosting)                             
struct __FILE { 
	int handle; 
}; 

FILE __stdout;         
void _sys_exit(int x) 
{ 
	x = x; 
} 

#ifdef CFG_UART_ENABLE
uint8_t uart1_buff[UART_BUFF_LEN];
uint8_t uart2_buff[UART_BUFF_LEN];
uint8_t uart3_buff[UART_BUFF_LEN];
uint8_t uart4_buff[UART_BUFF_LEN];

#if (CFG_PRINTF_TO_UART == 1)
    extern UART_HandleTypeDef huart1;
#elif (CFG_PRINTF_TO_UART == 2)
    extern UART_HandleTypeDef huart2;
#elif (CFG_PRINTF_TO_UART == 3)
    extern UART_HandleTypeDef huart3;
#endif

void embedi_enable_uart1_interrupt(void)
{
    HAL_UART_Receive_IT(&huart1, uart1_buff, UART_BUFF_LEN);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t *buff = NULL;
    uint8_t len = UART_BUFF_LEN;
    uint16_t timeout = UART_TIMEOUT;

    if (huart->Instance == USART1) {
        buff = uart1_buff;
    } else if (huart->Instance == USART2) {
        buff = uart2_buff;
    } else {
        buff = uart3_buff;
    }
    /* do something*/
    
    if (buff) {
        HAL_UART_Transmit(huart, buff, len, timeout);
        /* enable uart receive interrupt. keep it*/
        HAL_UART_Receive_IT(huart, buff, len);
    }

}

int fputc(int ch, FILE *f)
{
    USART_TypeDef *uart = NULL;

#if (CFG_PRINTF_TO_UART == 1)
    uart = USART1;
#elif (CFG_PRINTF_TO_UART == 2)
    uart = USART2;
#elif (CFG_PRINTF_TO_UART == 3)
    uart = USART3;
#else
    uart = NULL;
#endif

    if (uart) {
        while ((uart->SR & 0X40) == 0); 
        uart->DR = (uint8_t) ch;
    }
    return ch;
}
#else
int fputc(int ch, FILE *f)
{
    return ch;
}
#endif /* CFG_UART_ENABLE */
