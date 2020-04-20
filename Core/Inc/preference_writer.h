/*
 * preference_writer.h
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */

#ifndef INC_PREFERENCE_WRITER_H_
#define INC_PREFERENCE_WRITER_H_


#ifndef __PREFERENCE_WRITER_H
#define __PREFERENCE_WRITER_H

#include "FlashWriter.h"

class PreferenceWriter {
public:
    PreferenceWriter(uint32_t sector);
    void open();
    bool ready();
    void write(int x, int index);
    void write(float x, int index);
    void flush();
    void load();
    void close();
private:
    FlashWriter *writer;
    uint32_t __sector;
    bool __ready;
};

extern PreferenceWriter prefs;

#endif


#endif /* INC_PREFERENCE_WRITER_H_ */
