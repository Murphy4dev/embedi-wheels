#ifndef __EMBEDI_WHEELS_H
#define __EMBEDI_WHEELS_H

#ifdef __cplusplus
extern "C" {
#endif

/* need to config specific gpio*/
#define KEY_IRQ_GPIO GPIO_PIN_5
#define IMU_IRQ_GPIO GPIO_PIN_12
void embedi_wheels_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_WHEELS_H */
