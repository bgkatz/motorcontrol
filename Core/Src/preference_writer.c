/*
 * preference_writer.c
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */


#include "preference_writer.h"
#include "flash_writer.h"
#include "user_config.h"

PreferenceWriter::PreferenceWriter(uint32_t sector) {
    writer = new FlashWriter(sector);
    __sector = sector;
    __ready = false;
}

void PreferenceWriter::open() {
    writer->open();
    __ready = true;
}

bool  PreferenceWriter::ready() {
    return __ready;
}

void PreferenceWriter::write(int x, int index) {
    __int_reg[index] = x;
}

void PreferenceWriter::write(float x, int index) {
    __float_reg[index] = x;
}

void PreferenceWriter::flush() {
    int offs;
    for (offs = 0; offs < 256; offs++) {
        writer->write(offs, __int_reg[offs]);
    }
    for (; offs < 320; offs++) {
        writer->write(offs, __float_reg[offs - 256]);
    }
    __ready = false;
}

void PreferenceWriter::load() {
    int offs;
    for (offs = 0; offs < 256; offs++) {
        __int_reg[offs] = flashReadInt(__sector, offs);
    }
    for(; offs < 320; offs++) {
        __float_reg[offs - 256] = flashReadFloat(__sector, offs);
    }
}

void PreferenceWriter::close() {
    __ready = false;
    writer->close();
}
