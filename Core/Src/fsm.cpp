/*
 * fsm.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#include "fsm.h"
#include "usart.h"
#include <stdio.h>
#include "PreferenceWriter.h"
#include "user_config.h"

 void run_fsm(FSMStruct fsmstate){
	 if(fsmstate.state_change){
		 printf("FSM State %d\r\n", fsmstate.state);
	 }

	 switch(fsmstate.state){
	 case REST_MODE:
		 break;
	 case CALIBRATION_MODE:
		 break;
	 case MOTOR_MODE:
		 break;
	 case SETUP_MODE:
		 break;
	 case ENCODER_MODE:
		 break;
	 case INIT_TEMP_MODE:
		 break;
	 }

 }



 void update_fsm(FSMStruct fsmstate, char fsm_input){

	if(fsm_input == 27){	// escape to exit do rest mode
		fsmstate.state = REST_MODE;
		fsmstate.state_change = 1;
		//return;
	}
	switch(fsmstate.state){
	case REST_MODE:
        switch (fsm_input){
            case 'c':
            	fsmstate.state = CALIBRATION_MODE;
            	fsmstate.state_change = 1;
                break;
            case 'm':
            	fsmstate.state = MOTOR_MODE;
            	fsmstate.state_change = 1;
                break;
            case 'e':
            	fsmstate.state = ENCODER_MODE;
            	fsmstate.state_change = 1;
                break;
            case 's':
            	fsmstate.state = SETUP_MODE;
            	fsmstate.state_change = 1;
                break;
            case 'z':
                //spi.SetMechOffset(0);
                //spi.Sample(DT);
                HAL_Delay(20);
                //M_OFFSET = spi.GetMechPosition();
                if (!prefs.ready()) prefs.open();
                    prefs.flush();                                                  // Write new prefs to flash
                    prefs.close();
                    prefs.load();
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
	printf("FSM State: %d\r\n", fsmstate.state);
 }
