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
    LOW, 
    HIHG, 
    RELEASE = HIHG
};

enum {
    I2C_SUCCESS, 
    I2C_ERROR,
    I2C_NACK,
    I2C_START_FAIL
};

void emebedi_i2c_test(void);

#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_I2C_H */
