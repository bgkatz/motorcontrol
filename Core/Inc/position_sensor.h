/*
 * position_sensor.h
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */

#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_


#include "structs.h"
#include "spi.h"

void ps_warmup(EncoderStruct * encoder, int n);
void ps_sample(EncoderStruct * encoder, float dt);
void ps_print(EncoderStruct * encoder, int dt_ms);

#endif /* INC_POSITION_SENSOR_H_ */
