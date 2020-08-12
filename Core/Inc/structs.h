/*
 * structs.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#ifndef STRUCTS_H
#define STRUCTS_H
#ifdef __cplusplus
extern "C" {
#endif



#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"

typedef struct{
    } GPIOStruct;

typedef struct{
    }COMStruct;


/* Global Structs */
extern ControllerStruct controller;
extern ObserverStruct observer;
extern COMStruct com;
extern FSMStruct state;
extern EncoderStruct comm_encoder;
extern DRVStruct drv;


#endif
