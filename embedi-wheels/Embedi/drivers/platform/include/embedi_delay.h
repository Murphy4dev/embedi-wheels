#ifndef __EMBEDI_DELAY_H
#define __EMBEDI_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif
void embedi_delay_us(uint32_t microsec);
void embedi_delay_ms(uint16_t millisec);
void embedi_get_systick(unsigned long *tick);
#ifdef __cplusplus
}
#endif

#endif /* __EMBEDI_DELAY_H */
