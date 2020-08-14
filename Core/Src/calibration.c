/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */


#include "calibration.h"
#include "hw_config.h"
#include "user_config.h"
#include <stdio.h>
#include "usart.h"
#include "math_ops.h"

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, PreferenceWriter *pr, int loop_count){
	/* Checks phase order, to ensure that positive Q current produces
	   torque in the positive direction wrt the position sensor */

	if(!cal->started){
		printf("Checking phase sign, pole pairs\r\n");
		cal->started = 1;
		cal->start_count = loop_count;
	}
	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < 1.0f){
        /* Set voltage angle to zero, wait for rotor position to settle */
        cal->theta_ref = 0;//W_CAL*cal->time;
        controller->v_d = V_CAL;
        controller->v_q = 0.0f;
        abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
        svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
        set_dtc(controller);
    	cal->theta_start = encoder->angle_multiturn[0];
    	return;
    }

    else if(cal->time < 1.0f+2.0f*PI_F/W_CAL){
    	cal->theta_ref = W_CAL*(cal->time-1.0f);
    	abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
    	svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
    	set_dtc(controller);
    	return;
    }


    controller->v_d = 0;
	controller->v_q = 0.0f;
	abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
	svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
	set_dtc(controller);

	float theta_end = encoder->angle_multiturn[0];
	cal->ppairs = round(2.0f*PI_F/fabsf(theta_end-cal->theta_start));

	if(cal->theta_start < theta_end){
		cal->phase_order = 0;
		printf("Phase order correct\r\n");
	}
	else{
		cal->phase_order = 1;
		printf("Swapping phase sign\r\n");
	}
    printf("Pole Pairs: %d\r\n", cal->ppairs);
    cal->done_ordering = 1;	// Finished checking phase order
}

void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, PreferenceWriter *pr){

}

void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, PreferenceWriter *pr){

}
