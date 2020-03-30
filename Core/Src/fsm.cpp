/*
 * fsm.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#include "fsm.h"
#include "usart.h"
#include <stdio.h>
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
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
		printf("Got some serial:  %d\r\n", Serial2RxBuffer[0]);
		state.state = REST_MODE;
		state.state_change = 1;
	}
	switch(state.state){
	case REST_MODE:
        switch (c){
            case 'c':
                state = CALIBRATION_MODE;
                state_change = 1;
                break;
            case 'm':
                state = MOTOR_MODE;
                state_change = 1;
                break;
            case 'e':
                state = ENCODER_MODE;
                state_change = 1;
                break;
            case 's':
                state = SETUP_MODE;
                state_change = 1;
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
                spi.SetMechOffset(M_OFFSET);
                printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", M_OFFSET);
                break;
            }
            }
		break;
	case SETUP_MODE:
		break;
	case ENCODER_MODE:
		break;
	case MOTOR_MODE:
		break;
	}
 }
