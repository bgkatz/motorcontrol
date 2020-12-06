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
#include <stdlib.h>
#include "usart.h"
#include "math_ops.h"

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Checks phase order, to ensure that positive Q current produces
	   torque in the positive direction wrt the position sensor */

	if(!cal->started){
		printf("Checking phase sign, pole pairs\r\n");
		cal->started = 1;
		cal->start_count = loop_count;
	}
	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        controller->v_d = V_CAL;
        controller->v_q = 0.0f;
        abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
        svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
        set_dtc(controller);
    	cal->theta_start = encoder->angle_multiturn[0];
    	return;
    }

    else if(cal->time < T1+2.0f*PI_F/W_CAL){
    	// rotate voltage vector through one electrical cycle
    	cal->theta_ref = W_CAL*(cal->time-T1);
    	abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
    	svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
    	set_dtc(controller);
    	return;
    }


    controller->v_d = 0.0f;
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
    PHASE_ORDER = cal->phase_order;
    PPAIRS = (float)cal->ppairs;
    cal->started = 0;
    cal->done_ordering = 1;	// Finished checking phase order
}

void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count,
	/* Calibrates e-zero and encoder nonliearity */
	int *error_arr, int *lut_arr){


	if(!cal->started){
			error_array = malloc(sizeof(int)*cal->ppairs*128);
			lut_array = malloc(sizeof(int)*cal->ppairs*128);
			printf("Starting offset cal and linearization\r\n");
			cal->started = 1;
			cal->start_count = loop_count;
			cal->next_sample_time = T1;
			cal->sample_count = 0;
		}

	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        controller->v_d = V_CAL;
        controller->v_q = 0.0f;
        abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
        svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
        set_dtc(controller);
    	cal->theta_start = encoder->angle_multiturn[0];
    	return;
    }
    else if (cal->time < T1+2.0f*PI_F*PPAIRS/W_CAL){
    	// rotate voltage vector through one mechanical rotation in the positive direction
		cal->theta_ref += W_CAL*DT;//(cal->time-T1);
		abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
		svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
		set_dtc(controller);
		if(cal->time > cal->next_sample_time){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error =  encoder->raw - count_ref;
			error_array[cal->sample_count] = error + ENC_CPR*(error<0);
			//printf("%d %d\r\n", count_ref,encoder->raw);
			cal->sample_count++;
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*128.0f);
		}
		return;
    }
	else if (cal->time < T1+4.0f*PI_F*PPAIRS/W_CAL){
		// rotate voltage vector through one mechanical rotation
		cal->theta_ref -= W_CAL*DT;//(cal->time-T1);
		abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
		svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
		set_dtc(controller);
		if((cal->time > cal->next_sample_time) && (cal->sample_count < cal->ppairs*128)){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;
			error_array[cal->sample_count] = (error_array[cal->sample_count] + (error+ENC_CPR*error<0))/2;
			cal->sample_count++;
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*128.0f);
		}
		return;
    }

    int ezero_mean;
	for(int i = 0; i<128*PPAIRS; i++){
		ezero_mean += error_array[i];
	}
	E_ZERO = ezero_mean/(128*PPAIRS);

    controller->v_d = 0.0f;
	controller->v_q = 0.0f;
	abc(cal->theta_ref, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
	svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation
	set_dtc(controller);
	cal->done_cal = 1;
}

void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){

}
