/*
 * fsm.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#include "fsm.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "user_config.h"
#include "hw_config.h"
#include "structs.h"
#include "foc.h"
#include "math_ops.h"
#include "position_sensor.h"
#include "drv8323.h"

 void run_fsm(FSMStruct * fsmstate){
	 /* run_fsm is run every commutation interrupt cycle */

	 /* state transition management */
	 if(fsmstate->next_state != fsmstate->state){
		 fsm_exit_state(fsmstate);		// safely exit the old state
		 if(fsmstate->ready){			// if the previous state is ready, enter the new state
			 fsmstate->state = fsmstate->next_state;
			 fsm_enter_state(fsmstate);
		 }
	 }

	 switch(fsmstate->state){
		 case MENU_MODE:
			 break;

		 case CALIBRATION_MODE:
			 if(!comm_encoder_cal.done_ordering){
				 order_phases(&comm_encoder, &controller, &comm_encoder_cal, controller.loop_count);
			 }
			 else if(!comm_encoder_cal.done_cal){
				 calibrate_encoder(&comm_encoder, &controller, &comm_encoder_cal, controller.loop_count);
			 }
			 else{
				 /* Exit calibration mode when done */
				 //for(int i = 0; i<128*PPAIRS; i++){printf("%d\r\n", error_array[i]);}
				 E_ZERO = comm_encoder_cal.ezero;
				 printf("E_ZERO: %d  %f\r\n", E_ZERO, TWO_PI_F*fmodf((comm_encoder.ppairs*(float)(-E_ZERO))/((float)ENC_CPR), 1.0f));
				 memcpy(&comm_encoder.offset_lut, comm_encoder_cal.lut_arr, sizeof(comm_encoder.offset_lut));
				 memcpy(&ENCODER_LUT, comm_encoder_cal.lut_arr, sizeof(comm_encoder_cal.lut_arr));
				 //for(int i = 0; i<128; i++){printf("%d\r\n", ENCODER_LUT[i]);}
				 if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
				 preference_writer_flush(&prefs);
				 preference_writer_close(&prefs);
				 preference_writer_load(prefs);
				 update_fsm(fsmstate, 27);
			 }

			 break;

		 case MOTOR_MODE:
			 /* If CAN has timed out, reset all commands */
			 if((CAN_TIMEOUT > 0 ) && (controller.timeout > CAN_TIMEOUT)){
				 zero_commands(&controller);
			 }
			 /* Otherwise, commutate */
			 else{
				 torque_control(&controller);
				 field_weaken(&controller);
				 commutate(&controller, &comm_encoder);
			 }
			 controller.timeout ++;
			 break;

		 case SETUP_MODE:
			 break;

		 case ENCODER_MODE:
			 ps_print(&comm_encoder, 100);
			 break;

		 case INIT_TEMP_MODE:
			 break;
	 }

 }

 void fsm_enter_state(FSMStruct * fsmstate){
	 /* Called when entering a new state
	  * Do necessary setup   */

		switch(fsmstate->state){
				case MENU_MODE:
				//printf("Entering Main Menu\r\n");
				enter_menu_state();
				break;
			case SETUP_MODE:
				//printf("Entering Setup\r\n");
				enter_setup_state();
				break;
			case ENCODER_MODE:
				//printf("Entering Encoder Mode\r\n");
				break;
			case MOTOR_MODE:

				//printf("Entering Motor Mode\r\n");
				HAL_GPIO_WritePin(LED, GPIO_PIN_SET );
				reset_foc(&controller);
				drv_enable_gd(drv);
				break;
			case CALIBRATION_MODE:
				//printf("Entering Calibration Mode\r\n");
				/* zero out all calibrations before starting */

				comm_encoder_cal.done_cal = 0;
				comm_encoder_cal.done_ordering = 0;
				comm_encoder_cal.started = 0;
				comm_encoder.e_zero = 0;
				memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));
				drv_enable_gd(drv);
				break;

		}
 }

 void fsm_exit_state(FSMStruct * fsmstate){
	 /* Called when exiting the current state
	  * Do necessary cleanup  */

		switch(fsmstate->state){
			case MENU_MODE:
				//printf("Leaving Main Menu\r\n");
				fsmstate->ready = 1;
				break;
			case SETUP_MODE:
				//printf("Leaving Setup Menu\r\n");
				fsmstate->ready = 1;
				break;
			case ENCODER_MODE:
				//printf("Leaving Encoder Mode\r\n");
				fsmstate->ready = 1;
				break;
			case MOTOR_MODE:
				/* Don't stop commutating if there are high currents or FW happening */
				if( (fabs(controller.i_q_filt)<1.0f) && (fabs(controller.i_d_filt)<1.0f) ){
					fsmstate->ready = 1;
					drv_disable_gd(drv);
					reset_foc(&controller);
					//printf("Leaving Motor Mode\r\n");
					HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );
				}
				zero_commands(&controller);		// Set commands to zero
				break;
			case CALIBRATION_MODE:
				//printf("Exiting Calibration Mode\r\n");
				drv_disable_gd(drv);
				//free(error_array);
				//free(lut_array);

				fsmstate->ready = 1;
				break;
		}

 }

 void update_fsm(FSMStruct * fsmstate, char fsm_input){
	 /*update_fsm is only run when new state-change information is received
	  * on serial terminal input or CAN input
	  */
	if(fsm_input == MENU_CMD){	// escape to exit do rest mode
		fsmstate->next_state = MENU_MODE;
		fsmstate->ready = 0;
		return;
	}
	switch(fsmstate->state){
		case MENU_MODE:
			switch (fsm_input){
				case CAL_CMD:
					fsmstate->next_state = CALIBRATION_MODE;
					fsmstate->ready = 0;
					break;
				case MOTOR_CMD:
					fsmstate->next_state = MOTOR_MODE;
					fsmstate->ready = 0;
					break;
				case ENCODER_CMD:
					fsmstate->next_state = ENCODER_MODE;
					fsmstate->ready = 0;
					break;
				case SETUP_CMD:
					fsmstate->next_state = SETUP_MODE;
					fsmstate->ready = 0;
					break;
				case ZERO_CMD:
					comm_encoder.m_zero = 0;
					ps_sample(&comm_encoder, DT);
					HAL_Delay(20);
					M_ZERO = comm_encoder.count;
					//if (!prefs.ready()) prefs.open();
					//    prefs.flush();                                                  // Write new prefs to flash
					//    prefs.close();
					//    prefs.load();
					//spi.SetMechOffset(M_OFFSET);
					printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", M_ZERO);
					break;
				}
			break;
		case SETUP_MODE:
			if(fsm_input == ENTER_CMD){
				process_user_input(fsmstate);
				break;
			}
			if(fsmstate->bytecount == 0){fsmstate->cmd_id = fsm_input;}
			else{
				fsmstate->cmd_buff[fsmstate->bytecount-1] = fsm_input;
				//fsmstate->bytecount = fsmstate->bytecount%(sizeof(fsmstate->cmd_buff)/sizeof(fsmstate->cmd_buff[0])); // reset when buffer is full
			}
			fsmstate->bytecount++;
			/* If enter is typed, process user input */

			break;

		case ENCODER_MODE:
			break;
		case MOTOR_MODE:
			break;
	}
	//printf("FSM State: %d  %d\r\n", fsmstate.state, fsmstate.state_change);
 }


 void enter_menu_state(void){
	    //drv.disable_gd();
	    //reset_foc(&controller);
	    //gpio.enable->write(0);
	    printf("\n\r\n\r");
	    printf(" Commands:\n\r");
	    printf(" m - Motor Mode\n\r");
	    printf(" c - Calibrate Encoder\n\r");
	    printf(" s - Setup\n\r");
	    printf(" e - Display Encoder\n\r");
	    printf(" z - Set Zero Position\n\r");
	    printf(" esc - Exit to Menu\n\r");

	    //gpio.led->write(0);
 }

 void enter_setup_state(void){
	    printf("\r\n Configuration Options \n\r");
	    printf(" %-4s %-31s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");
	    printf("\r\n Motor:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %.3f\n\r", "g", "Gear Ratio", "0", "-", GR);
	    printf(" %-4s %-31s %-5s %-6s %.5f\n\r", "k", "Torque Constant (N-m/A)", "0", "-", KT);
	    printf("\r\n Control:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "l", "Current Limit (A)", "0.0", "40.0", I_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "p", "Max Position Setpoint (rad)", "-", "-", P_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "v", "Max Velocity Setpoint (rad)/s", "-", "-", V_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "x", "Max Position Gain (N-m/rad)", "0.0", "1000.0", KP_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "d", "Max Velocity Gain (N-m/rad/s)", "0.0", "5.0", KD_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
	    //printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Temp Cutoff (C) (0 = none)", "0", "150", TEMP_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "c", "Continuous Current (A)", "0.0", "40.0", I_MAX_CONT);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "a", "Calibration Current (A)", "0.0", "20.0", I_CAL);
	    printf("\r\n CAN:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
	    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
	    printf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
	    printf(" \n\r To change a value, type 'prefix''value''ENTER'\n\r e.g. 'b1000''ENTER'\r\n ");
	    printf("VALUES NOT ACTIVE UNTIL POWER CYCLE! \n\r\n\r");
 }

 void process_user_input(FSMStruct * fsmstate){
	 /* Collects user input from serial (maybe eventually CAN) and updates settings */

	 switch (fsmstate->cmd_id){
		 case 'b':
			 I_BW = fmaxf(fminf(atof(fsmstate->cmd_buff), 2000.0f), 100.0f);
			 printf("I_BW set to %f\r\n", I_BW);
			 break;
		 case 'i':
			 CAN_ID = atoi(fsmstate->cmd_buff);
			 printf("CAN_ID set to %d\r\n", CAN_ID);
			 break;
		 case 'm':
			 CAN_MASTER = atoi(fsmstate->cmd_buff);
			 printf("CAN_MASTER set to %d\r\n", CAN_MASTER);
			 break;
		 case 'l':
			 I_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
			 printf("I_MAX set to %f\r\n", I_MAX);
			 break;
		 case 'f':
			 I_FW_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 33.0f), 0.0f);
			 printf("I_FW_MAX set to %f\r\n", I_FW_MAX);
			 break;
		 case 't':
			 CAN_TIMEOUT = atoi(fsmstate->cmd_buff);
			 printf("CAN_TIMEOUT set to %d\r\n", CAN_TIMEOUT);
			 break;
		 case 'h':
			 TEMP_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 150.0f), 0.0f);
			 printf("TEMP_MAX set to %f\r\n", TEMP_MAX);
			 break;
		 case 'c':
			 I_MAX_CONT = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
			 printf("I_MAX_CONT set to %f\r\n", I_MAX_CONT);
			 break;
		 case 'a':
			 I_CAL = fmaxf(fminf(atof(fsmstate->cmd_buff), 20.0f), 0.0f);
			 printf("I_CAL set to %f\r\n", I_CAL);
			 break;
		 case 'g':
			 GR = fmaxf(atof(fsmstate->cmd_buff), .001f);	// Limit prevents divide by zero if user tries to enter zero
			 printf("GR set to %f\r\n", GR);
			 break;
		 case 'k':
			 KT = fmaxf(atof(fsmstate->cmd_buff), 0.0001f);	// Limit prevents divide by zero.  Seems like a reasonable LB?
			 printf("KT set to %f\r\n", KT);
			 break;
		 case 'x':
			 KP_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 printf("KP_MAX set to %f\r\n", KP_MAX);
			 break;
		 case 'd':
			 KD_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 printf("KD_MAX set to %f\r\n", KD_MAX);
			 break;
		 case 'p':
			 P_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 P_MIN = -P_MAX;
			 printf("P_MAX set to %f\r\n", P_MAX);
			 break;
		 case 'v':
			 V_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 V_MIN = -V_MAX;
			 printf("V_MAX set to %f\r\n", V_MAX);
			 break;
		 default:
			 printf("\n\r '%c' Not a valid command prefix\n\r\n\r", fsmstate->cmd_buff);
			 break;

		 }

	 /* Write new settings to flash */

	 if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
	 preference_writer_flush(&prefs);
	 preference_writer_close(&prefs);
	 preference_writer_load(prefs);

	 enter_setup_state();

	 fsmstate->bytecount = 0;
	 fsmstate->cmd_id = 0;
	 memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
 }

 void enter_motor_mode(void){

 }

