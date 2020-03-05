#include "stm32f4xx_flash.h"
#include "FlashWriter.h"

FlashWriter::FlashWriter(int sector) {
    if (sector > 7) sector = 7;
    __sector = sector;
    __base = __SECTOR_ADDRS[sector];
    __ready = false;
}

bool FlashWriter::ready() {
    return __ready;
}

void FlashWriter::open() {
    FLASH_Unlock();
    FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    FLASH_EraseSector(__SECTORS[__sector], VoltageRange_3);
    __ready = true;
}

void FlashWriter::write(uint32_t index, int x) {
    union {int a; uint32_t b;};
    a = x;
    FLASH_ProgramWord(__base + 4 * index, b);
}

void FlashWriter::write(uint32_t index, unsigned int x) {
    FLASH_ProgramWord(__base + 4 * index, x);
}

void FlashWriter::write(uint32_t index, float x) {
    union {float a; uint32_t b;};
    a = x;
    FLASH_ProgramWord(__base + 4 * index, b);
}

void FlashWriter::close() {
    FLASH_Lock();
    __ready = false;
}

int flashReadInt(uint32_t sector, uint32_t index) {
    return *(int*) (__SECTOR_ADDRS[sector] + 4 * index);
}   

uint32_t flashReadUint(uint32_t sector, uint32_t index) {
    return *(uint32_t*) (__SECTOR_ADDRS[sector] + 4 * index);
}   

float flashReadFloat(uint32_t sector, uint32_t index) {
    return *(float*) (__SECTOR_ADDRS[sector] + 4 * index);
}   
