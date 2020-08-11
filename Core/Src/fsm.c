/*
 * fsm.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#include "fsm.h"
#include "usart.h"
#include <stdio.h>
//#include "PreferenceWriter.h"
#include "user_config.h"
#include "hw_config.h"
#include "structs.h"
#include "foc.h"
#include "math_ops.h"
#include "position_sensor.h"
#include "drv8323.h"

 void run_fsm(FSMStruct * fsmstate){
	 /* run_fsm is run every commutation interrupt cycle */

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

		 break;

	 case MOTOR_MODE:

		 // get latest commands
		 // torque_control
		 commutate(&controller, &observer, &comm_encoder);

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
			printf("Entering Main Menu\r\n");
			enter_menu_state();
			break;
		case SETUP_MODE:
			printf("Entering Setup\r\n");
			enter_setup_state();
			break;
		case ENCODER_MODE:
			printf("Entering Encoder Mode\r\n");
			break;
		case MOTOR_MODE:
			printf("Entering Motor Mode\r\n");
			HAL_GPIO_WritePin(LED, GPIO_PIN_SET );
			reset_foc(&controller);
			drv_enable_gd(drv);
			break;
		}
 }

 void fsm_exit_state(FSMStruct * fsmstate){
	 /* Called when exiting the current state
	  * Do necessary cleanup  */

		switch(fsmstate->state){
		case MENU_MODE:
			printf("Leaving Main Menu\r\n");
			fsmstate->ready = 1;
			break;
		case SETUP_MODE:
			printf("Leaving Setup Menu\r\n");
			fsmstate->ready = 1;
			break;
		case ENCODER_MODE:
			printf("Leaving Encoder Mode\r\n");
			fsmstate->ready = 1;
			break;
		case MOTOR_MODE:
			/* Don't stop commutating if there are high currents or FW happening */
			if( (fabs(controller.i_q_filt)<1.0f) && (fabs(controller.i_d_filt)<1.0f) ){
				fsmstate->ready = 1;
				drv_disable_gd(drv);
				reset_foc(&controller);
				printf("Leaving Motor Mode\r\n");
				HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );
			}
			zero_commands(&controller);		// Set commands to zero
			break;
		}
 }

 void update_fsm(FSMStruct * fsmstate, char fsm_input){
	 /*update_fsm is only run when new state-change information is received
	  * on serial terminal input or CAN input
	  */
	if(fsm_input == 27){	// escape to exit do rest mode
		fsmstate->next_state = MENU_MODE;
		fsmstate->ready = 0;
		return;
	}
	switch(fsmstate->state){
	case MENU_MODE:
        switch (fsm_input){
            case 'c':
            	fsmstate->next_state = CALIBRATION_MODE;
            	fsmstate->ready = 0;
                break;
            case 'm':
            	fsmstate->next_state = MOTOR_MODE;
            	fsmstate->ready = 0;
                break;
            case 'e':
            	fsmstate->next_state = ENCODER_MODE;
            	fsmstate->ready = 0;
                break;
            case 's':
            	fsmstate->next_state = SETUP_MODE;
            	fsmstate->ready = 0;
                break;
            case 'z':
                //spi.SetMechOffset(0);
                //spi.Sample(DT);
                HAL_Delay(20);
                //M_OFFSET = spi.GetMechPosition();
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
	    printf("\n\r\n\r\n\r");
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
	    printf("\n\r\n\r Configuration Options \n\r\n\n");
	    printf(" %-4s %-31s %-5s %-6s %-2s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
	    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
	    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "l", "Current Limit (A)", "0.0", "40.0", I_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
	    printf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Temp Cutoff (C) (0 = none)", "0", "150", TEMP_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "c", "Continuous Current (A)", "0", "40.0", I_MAX_CONT);
	    printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
 }


 void enter_motor_mode(void){

 }

