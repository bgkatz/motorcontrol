/*
 * fsm.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */


#ifndef INC_FSM_H_
#define INC_FSM_H_
#ifdef __cplusplus
extern "C" {
#endif




#define MENU_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6

#include "structs.h"

 void run_fsm(FSMStruct* fsmstate);
 void update_fsm(FSMStruct * fsmstate, char fsm_input);
 void fsm_enter_state(FSMStruct * fsmstate);
 void fsm_exit_state(FSMStruct * fsmstate);
 void enter_menu_state(void);
 void enter_setup_state(void);
 void enter_motor_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
