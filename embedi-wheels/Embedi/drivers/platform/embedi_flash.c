#include <stdio.h>
#include "embedi_config.h"
#include "embedi_flash.h"
#include "embedi_test.h"

static uint16_t _read_data(uint32_t addr)
{
    return *(volatile uint16_t *)addr;
}

static void _write_data(uint32_t addr, uint16_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, buf[i]);
        addr += 2;
    }
}

void embedi_read_flash(uint32_t addr, uint16_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++) {
        buf[i] = _read_data(addr);
        addr += 2;
    }
}

uint16_t g_flash_buf[_PAGE_SIZE / 2];
void embedi_write_flash(uint32_t addr, uint16_t *buf, uint16_t len)
{
    FLASH_EraseInitTypeDef flash_eraseop;
    uint32_t erase_addr = 0;
    uint32_t page_positon = 0;
    uint32_t page_offset = 0;
    uint16_t in_page_offset = 0;
    uint16_t in_page_remain = 0;
    uint16_t i = 0;
    uint8_t need_to_erase = 0;

    if (addr < _FLASH_BASE ||
        (addr >= (_FLASH_BASE + _FLASH_SIZE))) {
        printf("error addr:0x%x \n", addr);
        return;
    }

    HAL_FLASH_Unlock();
    page_offset = addr - _FLASH_BASE;
    page_positon = page_offset / _PAGE_SIZE;
    in_page_offset = (page_offset % _PAGE_SIZE) / 2;
    in_page_remain = _PAGE_SIZE / 2 - in_page_offset;
    if (len <= in_page_remain) {
        in_page_remain = len;
    }

    while (1) {
        embedi_read_flash(page_positon * _PAGE_SIZE + _FLASH_BASE, g_flash_buf, _PAGE_SIZE / 2);
        for (i = 0; i < in_page_remain; i++) {
            if (g_flash_buf[in_page_offset + i] != 0XFFFF) {
                need_to_erase = 1;
                break;
            }
        }
        if (need_to_erase) {
            flash_eraseop.TypeErase = FLASH_TYPEERASE_PAGES;
            flash_eraseop.Banks = FLASH_BANK_1;
            flash_eraseop.NbPages = 1;
            flash_eraseop.PageAddress = page_positon * _PAGE_SIZE + _FLASH_BASE;
            HAL_FLASHEx_Erase(&flash_eraseop, &erase_addr);

            for (i = 0; i < in_page_remain; i++) {
                g_flash_buf[i + in_page_offset] = buf[i];
            }
            _write_data(page_positon * _PAGE_SIZE + _FLASH_BASE, g_flash_buf, _PAGE_SIZE / 2);
        } else {
            _write_data(addr, buf, in_page_remain);
        }
        if (len == in_page_remain) {
            break; /* finish */
        } else {   /* writting */
            page_positon++;
            in_page_offset = 0;
            buf += in_page_remain;
            addr += in_page_remain * 2;
            len -= in_page_remain;
            if (len > (_PAGE_SIZE / 2)) {
                in_page_remain = _PAGE_SIZE / 2;
            } else {
                in_page_remain = len;
            }
        }
    }

    HAL_FLASH_Lock();
}

extern int run_test;
void embedi_flash_test(void)
{
    uint8_t write_buf[] = {"10123456789"};
    uint8_t read_buf[12];
    uint8_t len = sizeof(write_buf) / 2 + ((sizeof(write_buf) % 2) ? 1 : 0);

    if (run_test == FLASH_WRITE) {
        printf("write %s %d %d \n", write_buf, len, run_test);
        embedi_write_flash(IMU_ADDR, (uint16_t *)write_buf, len);
    } else if (run_test == FLASH_READ) {
        embedi_read_flash(IMU_ADDR, (uint16_t *)read_buf, len);
        printf("read %s %d %d \n", read_buf, len, run_test);
    }
}
