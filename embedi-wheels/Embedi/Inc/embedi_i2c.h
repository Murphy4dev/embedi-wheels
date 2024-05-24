#ifndef __EMBEDI_I2C_H
#define __EMBEDI_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

//#define EXTERNAL_PULL_UP

#define _SCK GPIO_PIN_8
#define _SDA GPIO_PIN_9
#define _SCK_PORT GPIOB
#define _SDA_PORT GPIOB

enum {
    I2C_SUCCESS, 
    I2C_ERROR,
    I2C_NACK,
    I2C_START_FAIL
};

void emebedi_i2c_test(void);
int embedi_i2c_read_byte(uint8_t addr, uint8_t reg,  uint8_t *buf);
int embedi_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data);
int embedi_i2c_read_block(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
int embedi_i2c_write_block(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_I2C_H */
