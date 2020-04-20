/*
 * fsm.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */


#ifndef INC_FSM_H_
#define INC_FSM_H_



#define REST_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6

#include "structs.h"

 void run_fsm(FSMStruct fsmstate);
 void update_fsm(FSMStruct fsmstate, char fsm_input);


#endif /* INC_FSM_H_ */
