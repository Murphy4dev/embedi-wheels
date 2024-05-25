#ifndef __EMBEDI_FLASH_H
#define __EMBEDI_FLASH_H

/* FLASH start address */
#define _FLASH_SIZE        0x10000         /* 64k */
#define _FLASH_BASE        0x08000000

 /* page size */
#if _FLASH_SIZE < 256 * 1024
#define _PAGE_SIZE   1024                /* 1K */
#else
#define _PAGE_SIZE   2048                /* 2K */
#endif

/* FLASH KEY VALUE */
#define _FLASH_KEY1        0X45670123
#define _FLASH_KEY2        0XCDEF89AB

/*embedi reserve space*/
#if _FLASH_SIZE < 256 * 1024
#define _RESERVE_SIZE   0X400                /* 1K */
#define _RESERVE_ADDR   (_FLASH_BASE+_FLASH_SIZE-_RESERVE_SIZE)
#else
#define _RESERVE_SIZE   0X800                /* 2K */
#define _RESERVE_ADDR   (_FLASH_BASE+_FLASH_SIZE-_RESERVE_SIZE)
#endif
/* imu calibration*/
#define IMU_SIZE 0X18 /*4 * 6 = 24 byte*/
#define IMU_OFFSET (_RESERVE_SIZE - IMU_SIZE)
#define IMU_ADDR (_RESERVE_ADDR + IMU_OFFSET) /* back */

              
void embedi_read_flash(uint32_t addr, uint16_t *buf, uint16_t len);
void embedi_write_flash(uint32_t addr, uint16_t *buf, uint16_t len);
void embedi_flash_test(void);
#endif
