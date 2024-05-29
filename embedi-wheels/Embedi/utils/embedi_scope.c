#include "embedi_config.h"
#include <stdio.h>
#include "embedi_uart.h"

#define CHANNEL_DATABUFF_LEN 4
#define CHANNEL_NUM 6
#define BUFF_LEN (CHANNEL_NUM * 4 + 2)
unsigned char g_buffer[BUFF_LEN];

static void _float2byte(float *data, unsigned char *buf)
{
    unsigned char *p = NULL;

    if (!data || !buf) {
        return;
    }
    p = (unsigned char *)data;

    buf[0] = p[0];
    buf[1] = p[1];
    buf[2] = p[2];
    buf[3] = p[3];
}

static void _data2buff(unsigned char *data, unsigned char index)
{
    if (!data) {
        return;
    }

    g_buffer[index] = data[0];
    g_buffer[index + 1] = data[1];
    g_buffer[index + 2] = data[2];
    g_buffer[index + 3] = data[3];
}

static int _find_channel_index(unsigned char channel)
{
    return (1 + CHANNEL_DATABUFF_LEN * (channel - 1));
}

void embedi_data_to_scope(float data, unsigned char channel)
{
    int index = _find_channel_index(channel);
    unsigned char buf[4];

    if ((channel > CHANNEL_NUM) || (channel == 0)) {
        printf("chanel invalid %d \n", channel);
        return;
    }
    g_buffer[0] = '$';
    g_buffer[BUFF_LEN - 1] = BUFF_LEN - 1;
    _float2byte(&data, buf);
    _data2buff(buf, index);
}

void embedi_scope_show(void)
{
    int i = 0;
    static int send = 0;

    /* imu 200hz need down sample to send to scope*/
    send = ++send % 2;

    if (send) {
        for (i = 0; i < BUFF_LEN; i++) {
            embedi_uart_send_byte(g_buffer[i]);
        }
    }
    // memset(g_buffer, 0, sizeof(g_buffer) * BUFF_LEN);
}
