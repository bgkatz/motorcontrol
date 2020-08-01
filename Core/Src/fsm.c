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

 void run_fsm(FSMStruct * fsmstate){
	 /* run_smf is run every commutation interrupt cycle */
	 if(fsmstate->state_change){
		 printf("FSM State %d\r\n", fsmstate->state);
	 }

	 switch(fsmstate->state){
	 case MENU_MODE:
		 if(fsmstate->state_change){
			 enter_menu_state();
		 	 fsmstate->state_change = 0;
		 	 }
		 break;
	 case CALIBRATION_MODE:
		 break;
	 case MOTOR_MODE:
		 break;
	 case SETUP_MODE:
		 if(fsmstate->state_change){
		 	enter_setup_state();
		 	fsmstate->state_change = 0;
		 	 }
		 break;
	 case ENCODER_MODE:
		 break;
	 case INIT_TEMP_MODE:
		 break;
	 }

 }



 void update_fsm(FSMStruct * fsmstate, char fsm_input){
	 /*update_fsm is only run when new state-change information is received
	  * on serial terminal input or CAN input
	  */
	if(fsm_input == 27){	// escape to exit do rest mode
		fsmstate->state = MENU_MODE;
		fsmstate->state_change = 1;
		return;
	}
	switch(fsmstate->state){
	case MENU_MODE:
        switch (fsm_input){
            case 'c':
            	fsmstate->state = CALIBRATION_MODE;
            	fsmstate->state_change = 1;
                break;
            case 'm':
            	fsmstate->state = MOTOR_MODE;
            	fsmstate->state_change = 1;
                break;
            case 'e':
            	fsmstate->state = ENCODER_MODE;
            	fsmstate->state_change = 1;
                break;
            case 's':
            	fsmstate->state = SETUP_MODE;
            	fsmstate->state_change = 1;
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
                printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", M_OFFSET);
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

 void calibrate(void){

 }

 void enter_torque_mode(void){

 }

