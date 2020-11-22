/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "position_sensor.h"
#include "user_config.h"
#include "foc.h"

#define V_CAL 	1.0f	// Calibration voltage
#define W_CAL 	10.0f	// Calibration speed in rad/s
#define T1		1.0f	// Cal settling period

typedef struct{
	uint8_t ppairs;									// number of pole pairs measured
	int offset;								// electrical zero position in counts
	float theta_ref;								// reference angle used for calibration
	int start_count;								// loop count at cal start
	uint8_t started;								// has cal started or not?
	float time;										// cal time
	float theta_start;								// cal start angle
	uint8_t phase_order;							// phase order correct (0) or swapped (1)
	uint8_t done_ordering, done_cal, done_rl;		// flags for different cals finished
	uint16_t sample_count;							// keep track of how many samples taken
	float next_sample_time;							// time to take next sample

} CalStruct;

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, int loop_count);
void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,
		int loop_count, int *error_arr, int *lut_arr);
void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,  int loop_count);

extern int *error_array;
extern int *lut_array;

#endif /* INC_CALIBRATION_H_ */
