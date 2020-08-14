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

#define V_CAL 1.0f		// Calibration voltage
#define W_CAL 3.0f		// Calibration speed in rad/s

typedef struct{
	uint8_t ppairs;									// number of pole pairs measured
	uint16_t offset;								// electrical zero position in counts
	float theta_ref;								// reference angle used for calibration
	int start_count;								// loop count at cal start
	uint8_t started;								// has cal started or not?
	float time;										// cal time
	float theta_start;								// cal start angle
	uint8_t phase_order;							// phase order correct (0) or swapped (1)
	uint16_t raw[N_LUT];							// encoder reading array for LUT generation
	uint16_t error[N_LUT];							// encodor error array for LUT generation
	uint8_t done_ordering, done_cal, done_rl;		// flags for different cals finished

} CalStruct;

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, PreferenceWriter *pr, int loop_count);
void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, PreferenceWriter *pr);
void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, PreferenceWriter *pr);

#endif /* INC_CALIBRATION_H_ */
