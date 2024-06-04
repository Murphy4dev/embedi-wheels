#include "embedi_config.h"
#include <stdio.h>
#include "embedi_module_init.h"

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
UART_HandleTypeDef *huart = &huart1;
#elif (CFG_PRINTF_TO_UART == 2)
extern UART_HandleTypeDef huart2;
UART_HandleTypeDef *huart = &huart2;
#elif (CFG_PRINTF_TO_UART == 3)
extern UART_HandleTypeDef huart3;
UART_HandleTypeDef *huart = &huart3;
#endif

void embedi_enable_uart1_interrupt(void)
{
    HAL_UART_Receive_IT(huart, uart1_buff, UART_BUFF_LEN * sizeof(uint8_t));
}

void embedi_uart_send_byte(const uint8_t data)
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
    while ((uart->SR & 0X40) == 0)
        ;
    uart->DR = data;
}

#include "embedi_system.h"
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
    embedi_set_run_state(buff[0]);
    if (buff) {
        HAL_UART_Transmit(huart, buff, len, timeout);
        /* enable uart receive interrupt. keep it*/
        HAL_UART_Receive_IT(huart, buff, len * sizeof(uint8_t));
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
        while ((uart->SR & 0X40) == 0)
            ;
        uart->DR = (uint8_t)ch;
    }
    return ch;
}
#else
int fputc(int ch, FILE *f)
{
    return ch;
}

void embedi_uart_send_byte(const uint8_t data)
{
    (void)data;
}
#endif /* CFG_UART_ENABLE */

void embedi_uart_init(void)
{
#ifdef CFG_UART_ENABLE
    embedi_enable_uart1_interrupt();
#endif
}

platform_init(embedi_uart_init);
