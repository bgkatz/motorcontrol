/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "position_sensor.h"
#include "preference_writer.h"
#include "foc.h"

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, PreferenceWriter *prefs);
void calibrate(EncoderStruct *encoder, ControllerStruct *controller, PreferenceWriter *prefs);

#endif /* INC_CALIBRATION_H_ */
