#ifndef FOC_H
#define FOC_H

#include "../structs.h"
#include "PositionSensor.h"
#include "mbed.h"
#include "hw_config.h"
#include "math.h"
#include "../math_ops.h"
#include "motor_config.h"
#include "current_controller_config.h"
#include "FastMath.h"
#include "user_config.h"

void abc(float theta, float d, float q, float *a, float *b, float *c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void svm(float v_bus, float u, float v, float w, int current_sector, float *dtc_u, float *dtc_v, float *dtc_w);
void zero_current(int *offset_1, int *offset_2);
void reset_foc(ControllerStruct *controller);
void reset_observer(ObserverStruct *observer);
void init_controller_params(ControllerStruct *controller);
void commutate(ControllerStruct *controller, ObserverStruct *observer, GPIOStruct *gpio, float theta);
void torque_control(ControllerStruct *controller);
void limit_current_ref (ControllerStruct *controller);
void update_observer(ControllerStruct *controller, ObserverStruct *observer);
void field_weaken(ControllerStruct *controller);
float linearize_dtc(ControllerStruct *controller, float dtc);
#endif
