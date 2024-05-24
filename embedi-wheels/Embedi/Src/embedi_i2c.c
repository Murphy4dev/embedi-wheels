#include "embedi_config.h"
#include "stdio.h"
#include "embedi_delay.h"
#include "embedi_i2c.h"
/*
when there is external pull-up R(Recomanded)
GPIO mode configuration
    1. SCK should be push-pull ouput
    2. SDA should be open drain

when there is not external pull-up R
GPIO mode configuration
    1. SCK should be push-pull ouput
    2. SDA should be push-pull ouput(need to change in/out mode)
*/
#define LOW GPIO_PIN_RESET
#define HIHG GPIO_PIN_SET
#define RELEASE HIHG

#ifdef CFG_I2C_GPIO_SIMULATION
#ifdef EXTERNAL_PULL_UP
#define IIC_SCL(status)                             \
    do {                                            \
        HAL_GPIO_WritePin(_SCK_PORT, _SCK, status); \
    } while (0)

#define IIC_SDA(status)                             \
    do {                                            \
        HAL_GPIO_WritePin(_SDA_PORT, _SDA, status); \
    } while (0)

#define REALSE_IIC_SDA()                          \
    do {                                          \
        HAL_GPIO_WritePin(_SDA_PORT, _SDA, HIHG); \
    } while (0)

#else /* EXTERNAL_PULL_UP */
#define IIC_SCL(status)                             \
    do {                                            \
        HAL_GPIO_WritePin(_SCK_PORT, _SCK, status); \
    } while (0)

#define IIC_SDA(status)                             \
    do {                                            \
        GPIO_InitTypeDef GPIO_InitStruct = {        \
            .Pin = _SDA,                            \
            .Mode = GPIO_MODE_OUTPUT_PP,            \
            .Pull = GPIO_NOPULL,                    \
            .Speed = GPIO_SPEED_FREQ_HIGH,          \
        };                                          \
        HAL_GPIO_Init(_SDA_PORT, &GPIO_InitStruct); \
        HAL_GPIO_WritePin(_SDA_PORT, _SDA, status); \
    } while (0)

#define REALSE_IIC_SDA()                            \
    do {                                            \
        GPIO_InitTypeDef GPIO_InitStruct = {        \
            .Pin = _SDA,                            \
            .Mode = GPIO_MODE_INPUT,                \
            .Pull = GPIO_PULLUP,                    \
            .Speed = GPIO_SPEED_FREQ_HIGH,          \
        };                                          \
        HAL_GPIO_Init(_SDA_PORT, &GPIO_InitStruct); \
    } while (0)
#endif /* EXTERNAL_PULL_UP */

#define READ_IIC_SCK() HAL_GPIO_ReadPin(_SDA_PORT, _SCK)
#define READ_IIC_SDA() HAL_GPIO_ReadPin(_SDA_PORT, _SDA)

#define DEBUG_IIC_SDA()                             \
    do {                                            \
        int status = -1;                            \
        status = HAL_GPIO_ReadPin(_SDA_PORT, _SDA); \
        printf("sda %d \n", status);                \
    } while (0)

#define DEBUG_IIC_SCK()                             \
    do {                                            \
        int status = -1;                            \
        status = HAL_GPIO_ReadPin(_SDA_PORT, _SCK); \
        printf("sck %x \n", status);                \
    } while (0)

#define RECORD_IIC_SWQUENCE(sda, sck) \
    do {                              \
        sda = sda << 1;               \
        sck = sck << 1;               \
        sda += READ_IIC_SDA();        \
        sck += READ_IIC_SCK();        \
    } while (0)

#define START_SCK_SEQUENCE (0X0E) // 0000 1110
#define START_SDA_SEQUENCE (0X0C) // 0000 1100
static int _start(void)
{
    uint8_t sda = 1;
    uint8_t sck = 1;

    IIC_SDA(HIHG);
    IIC_SCL(HIHG);
    embedi_delay_us(2);
    RECORD_IIC_SWQUENCE(sda, sck);

    /*START: when CLK is high,
    DATA change form high to low */
    IIC_SDA(LOW);
    embedi_delay_us(2);
    RECORD_IIC_SWQUENCE(sda, sck);

    /*hold i2c bus*/
    IIC_SCL(LOW);
    RECORD_IIC_SWQUENCE(sda, sck);

    if (sda != START_SDA_SEQUENCE ||
        sck != START_SCK_SEQUENCE) {
        printf("sck:0x%x sda:0x%x\n", sck, sda);
        return I2C_ERROR;
    }
    return I2C_SUCCESS;
}

#define STOP_SCK_SEQUENCE (0X0B) // 0000 1011
#define STOP_SDA_SEQUENCE (0X09) // 0000 1001
static int _stop(void)
{
    uint8_t sda = 1;
    uint8_t sck = 1;

    IIC_SCL(LOW);
    IIC_SDA(LOW);
    embedi_delay_us(2);
    RECORD_IIC_SWQUENCE(sda, sck);

    /*STOP: when CLK is high,
    DATA change form low to high */
    IIC_SCL(HIHG);
    RECORD_IIC_SWQUENCE(sda, sck);

    IIC_SDA(HIHG);
    embedi_delay_us(2);
    RECORD_IIC_SWQUENCE(sda, sck);

    if (sda != STOP_SDA_SEQUENCE ||
        sck != STOP_SCK_SEQUENCE) {
        printf("sck:0x%x sda:0x%x\n", sck, sda);
        return I2C_ERROR;
    }
    return I2C_SUCCESS;
}

static int _wait_ack(void)
{
    uint8_t waiting_time = 0;

    REALSE_IIC_SDA();
    embedi_delay_us(2);

    IIC_SCL(HIHG);
    embedi_delay_us(2);

    while (READ_IIC_SDA()) {
        waiting_time++;
        if (waiting_time > 50) {
            _stop();
            printf("nack\n");
            return I2C_NACK;
        }
        embedi_delay_us(1);
    }
    IIC_SCL(LOW);
    return I2C_SUCCESS;
}

static void _reponse_ack(void)
{
    IIC_SCL(LOW);
    IIC_SDA(LOW);
    embedi_delay_us(2);

    IIC_SCL(HIHG);
    embedi_delay_us(2);
    IIC_SCL(LOW);
}

static void _reponse_nack(void)
{
    IIC_SCL(LOW);
    IIC_SDA(HIHG);
    embedi_delay_us(2);

    IIC_SCL(HIHG);
    embedi_delay_us(2);
    IIC_SCL(LOW);
}

static void _send_byte(uint8_t data)
{
    uint8_t i = 0;

    /* prepare data */
    IIC_SCL(LOW);
    for (i = 0; i < 8; i++) {
        IIC_SDA((GPIO_PinState)((data & 0x80) >> 7));
        data <<= 1;
        embedi_delay_us(2);
        /*sck high data valid*/
        IIC_SCL(HIHG);
        embedi_delay_us(2);
        /*sck low data invalid*/
        IIC_SCL(LOW);
        embedi_delay_us(2);
    }
}

static uint8_t _read_byte(uint8_t is_send_ack)
{
    uint8_t i = 0;
    uint8_t receive = 0;

    /* release sda to be preparing to receive data */
    REALSE_IIC_SDA();
    for (i = 0; i < 8; i++) {
        IIC_SCL(LOW);
        embedi_delay_us(2);
        IIC_SCL(HIHG);
        receive <<= 1;
        if (READ_IIC_SDA()) {
            receive++;
        }
        embedi_delay_us(2);
    }
    if (is_send_ack)
        _reponse_ack();
    else
        _reponse_nack();

    return receive;
}

static int _i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (_start()) {
        return I2C_START_FAIL;
    }

    _send_byte(addr);
    if (_wait_ack()) {
        _stop();
        return I2C_NACK;
    }

    _send_byte(reg);
    if (_wait_ack()) {
        _stop();
        return I2C_NACK;
    }

    _start();
    _send_byte(addr + 1);
    if (_wait_ack()) {
        _stop();
        return I2C_NACK;
    }

    while (len) {
        if (len == 1)
            *buf = _read_byte(0);
        else
            *buf = _read_byte(1);
        buf++;
        len--;
    }
    _stop();
    return 0;
}

static int _i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    int i = 0;

    if (_start()) {
        return I2C_START_FAIL;
    }

    _send_byte(addr);
    if (_wait_ack()) {
        _stop();
        return I2C_NACK;
    }

    _send_byte(reg);
    if (_wait_ack()) {
        _stop();
        return I2C_NACK;
    }

    for (i = 0; i < len; i++) {
        _send_byte(data[i]);
        if (_wait_ack()) {
            _stop();
            return I2C_NACK;
        }
    }
    _stop();
    return I2C_SUCCESS;
}
#else
#if (CFG_I2C_INDEX == 1)
extern I2C_HandleTypeDef hi2c1;
#else
extern I2C_HandleTypeDef hi2c2;
#endif
#endif

int embedi_i2c_read_byte(uint8_t addr, uint8_t reg, uint8_t *buf)
{
#ifdef CFG_I2C_GPIO_SIMULATION
    return _i2c_read(addr, reg, 1, buf);
#else
    I2C_HandleTypeDef *hi2c = NULL;
#if (CFG_I2C_INDEX == 1)
    hi2c = &hi2c1;
#else
    hi2c = &hi2c2;
#endif
    return HAL_I2C_Mem_Read(hi2c, (uint16_t)addr, (uint16_t)reg, 1, buf, 1, 100);
#endif
}

int embedi_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buff = data;
#ifdef CFG_I2C_GPIO_SIMULATION
    return _i2c_write(addr, reg, 1, &buff);
#else
    I2C_HandleTypeDef *hi2c = NULL;
#if (CFG_I2C_INDEX == 1)
    hi2c = &hi2c1;
#else
    hi2c = &hi2c2;
#endif
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)addr, (uint16_t)reg, 1, &buff, 1, 100);
#endif
}

int embedi_i2c_read_block(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
#ifdef CFG_I2C_GPIO_SIMULATION
    return _i2c_read(addr, reg, len, buf);
#else
    I2C_HandleTypeDef *hi2c = NULL;
#if (CFG_I2C_INDEX == 1)
    hi2c = &hi2c1;
#else
    hi2c = &hi2c2;
#endif
    return HAL_I2C_Mem_Read(hi2c, (uint16_t)addr, (uint16_t)reg, 1, buf, (uint16_t)len, 100);
#endif
}

int embedi_i2c_write_block(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
#ifdef CFG_I2C_GPIO_SIMULATION
    return _i2c_write(addr, reg, len, data);
#else
    I2C_HandleTypeDef *hi2c = NULL;
#if (CFG_I2C_INDEX == 1)
    hi2c = &hi2c1;
#else
    hi2c = &hi2c2;
#endif
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)addr, (uint16_t)reg, 1, data, (uint16_t)len, 100);
#endif
}

extern int run_i2c_test;
void emebedi_i2c_test(void)
{
    int ret = 0;
    uint8_t data = 0;

    if (run_i2c_test) {
        run_i2c_test = 0;
        ret = embedi_i2c_read_block(0xD0, 0x75, 1, &data);
        printf("%d 0x%x\r\n", ret, data);
    }
}
