#include "stm32f4xx_flash.h"
#include "flash_writer.h"
/*
FlashWriter::FlashWriter(int sector) {
    if (sector > 7) sector = 7;
    __sector = sector;
    __base = __SECTOR_ADDRS[sector];
    __ready = false;
}
*/

void flash_writer_init(FlashWriter fw, sector) {
	if(sector>7) sector = 7;
	fw.sector = sector;
	fw.base = __SECTOR_ADDRS[sector];
	fw.ready = false;
}
bool flash_writer_ready(FlashWriter fw) {
    return fw.ready;
}

void flash_writer_open(FlashWriter fw) {
    FLASH_Unlock();
    FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_EraseSector(__SECTORS[__sector], VoltageRange_3);
    fw.ready = true;
}

void flash_writer_write(FlashWriter fw, uint32_t index, int x) {
    union {int a; uint32_t b;};
    a = x;
    FLASH_ProgramWord(fw.base + 4 * index, b);
}

void flash_writer_write(FlashWriter fw, uint32_t index, unsigned int x) {
    FLASH_ProgramWord(fw.base + 4 * index, x);
}

void flash_writer_write(FlashWriter fw, uint32_t index, float x) {
    union {float a; uint32_t b;};
    a = x;
    FLASH_ProgramWord(fw.base + 4 * index, b);
}

void flash_writer_close(FlashWriter fw) {
    FLASH_Lock();
    fw.ready = false;
}

int flash_read_int(uint32_t sector, uint32_t index) {
    return *(int*) (__SECTOR_ADDRS[sector] + 4 * index);
}

uint32_t flash_read_uint(uint32_t sector, uint32_t index) {
    return *(uint32_t*) (__SECTOR_ADDRS[sector] + 4 * index);
}

float flash_read_float(uint32_t sector, uint32_t index) {
    return *(float*) (__SECTOR_ADDRS[sector] + 4 * index);
}
