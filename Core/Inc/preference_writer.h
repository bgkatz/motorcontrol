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

#include "flash_writer.h"
#include "stdbool.h"

/*
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
*/

/*
PreferenceWriter::PreferenceWriter(uint32_t sector) {
    writer = new FlashWriter(sector);
    __sector = sector;
    __ready = false;
}
*/
typedef struct{
	FlashWriter fw;
	uint32_t sector;
	bool ready;
}PreferenceWriter;

void preference_writer_init(PreferenceWriter * pr, uint32_t sector);
void preference_writer_open(PreferenceWriter * pr);
bool preference_writer_ready(PreferenceWriter pr);
void preference_writer_write_int(int x, int index);
void preference_writer_write_float(float x, int index);
void preference_writer_flush(PreferenceWriter * pr);
void preference_writer_load(PreferenceWriter pr);
void preference_writer_close(PreferenceWriter *pr);

extern PreferenceWriter prefs;


#endif


#endif /* INC_PREFERENCE_WRITER_H_ */
