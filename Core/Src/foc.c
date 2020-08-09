/*
 * foc.c
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#include "foc.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

void set_dtc(ControllerStruct *controller){
	/* Output duty cycle from controller to the pwm timer */

	if(INVERT_DTC){
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*(1.0f-controller->dtc_u));
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*(1.0f-controller->dtc_v));
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*(1.0f-controller->dtc_w));
	}
	else{
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*(controller->dtc_u));
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*(controller->dtc_v));
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*(controller->dtc_w));
	}
}

void abc( float theta, float d, float q, float *a, float *b, float *c){
    /* Inverse DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1 */

    float cf = cosf(theta);
    float sf = sinf(theta);

    *a = cf*d - sf*q;
    *b = (SQRT3_2*sf-.5f*cf)*d - (-SQRT3_2*cf-.5f*sf)*q;
    *c = (-SQRT3_2*sf-.5f*cf)*d - (SQRT3_2*cf-.5f*sf)*q;
    }


void dq0(float theta, float a, float b, float c, float *d, float *q){
    /* DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1*/

    float cf = cosf(theta);
    float sf = sinf(theta);

    *d = 0.6666667f*(cf*a + (SQRT3_2*sf-.5f*cf)*b + (-SQRT3_2*sf-.5f*cf)*cf);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-SQRT3_2*cf-.5f*sf)*b - (SQRT3_2*cf-.5f*sf)*cf);

    }

void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w){
    /* Space Vector Modulation
     u,v,w amplitude = v_bus for full modulation depth */

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;


    *dtc_u = fminf(fmaxf((.5f*(u -v_offset)/(v_bus*(DTC_MAX-DTC_MIN)) + (DTC_MAX+DTC_MIN)*.5f ), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf((.5f*(v -v_offset)/(v_bus*(DTC_MAX-DTC_MIN)) + (DTC_MAX+DTC_MIN)*.5f ), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf((.5f*(w -v_offset)/(v_bus*(DTC_MAX-DTC_MIN)) + (DTC_MAX+DTC_MIN)*.5f ), DTC_MIN), DTC_MAX);

    }

void zero_current(ControllerStruct *controller){
	/* Measure zero-current ADC offset */

    int adc_b_offset = 0;
    int adc_c_offset = 0;
    int n = 1000;
    controller->dtc_u = 1.0f;
    controller->dtc_v = 1.0f;
    controller->dtc_w = 1.0f;
    set_dtc(controller);
    for (int i = 0; i<n; i++){               // Average n samples
    	HAL_ADC_Start(&ADC_CH_MAIN);
    	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    	adc_b_offset +=  HAL_ADC_GetValue(&ADC_CH_IB);
    	adc_c_offset += HAL_ADC_GetValue(&ADC_CH_IC);
     }
    controller->adc_b_offset = adc_b_offset/n;
    controller->adc_c_offset = adc_c_offset/n;

    }

void init_controller_params(ControllerStruct *controller){
    /*
	controller->ki_d = KI_D;
    controller->ki_q = KI_Q;
    controller->k_d = K_SCALE*I_BW;
    controller->k_q = K_SCALE*I_BW;
    controller->alpha = 1.0f - 1.0f/(1.0f - DT*I_BW*2.0f*PI);
    for(int i = 0; i<128; i++)
    {
        controller->inverter_tab[i] = 1.0f + 1.2f*exp(-0.0078125f*i/.032f);
    }
    */
    }

void reset_foc(ControllerStruct *controller){

	TIM_PWM.Instance->CCR3 = ((TIM_PWM.Instance->ARR)>>1)*(0.5f);
	TIM_PWM.Instance->CCR1 = ((TIM_PWM.Instance->ARR)>>1)*(0.5f);
	TIM_PWM.Instance->CCR2 = ((TIM_PWM.Instance->ARR)>>1)*(0.5f);
    controller->i_d_ref = 0;
    controller->i_q_ref = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
    controller->otw_flag = 0;

    }

void reset_observer(ObserverStruct *observer){
/*
    observer->temperature = 25.0f;
    observer->temp_measured = 25.0f;
    //observer->resistance = .1f;
*/
}

void limit_current_ref (ControllerStruct *controller){
	/*
    float i_q_max_limit = (0.5774f*controller->v_bus - controller->dtheta_elec*WB)/R_PHASE;
    float i_q_min_limit = (-0.5774f*controller->v_bus - controller->dtheta_elec*WB)/R_PHASE;
    controller->i_q_ref = fmaxf(fminf(i_q_max_limit, controller->i_q_ref), i_q_min_limit);
    */
    }

void update_observer(ControllerStruct *controller, ObserverStruct *observer)
{
	/*
    /// Update observer estimates ///
    // Resistance observer //
    // Temperature Observer //
    observer->delta_t = (float)observer->temperature - T_AMBIENT;
    float i_sq = controller->i_d*controller->i_d + controller->i_q*controller->i_q;
    observer->q_in = (R_NOMINAL*1.5f)*(1.0f + .00393f*observer->delta_t)*i_sq;
    observer->q_out = observer->delta_t*R_TH;
    observer->temperature += (INV_M_TH*DT)*(observer->q_in-observer->q_out);

    //float r_d = (controller->v_d*(DTC_MAX-DTC_MIN) + SQRT3*controller->dtheta_elec*(L_Q*controller->i_q))/(controller->i_d*SQRT3);
    float r_q = (controller->v_q*(DTC_MAX-DTC_MIN) - SQRT3*controller->dtheta_elec*(L_D*controller->i_d + WB))/(controller->i_q*SQRT3);
    observer->resistance = r_q;//(r_d*controller->i_d + r_q*controller->i_q)/(controller->i_d + controller->i_q); // voltages more accurate at higher duty cycles

    //observer->resistance = controller->v_q/controller->i_q;
    if(isnan(observer->resistance) || isinf(observer->resistance)){observer->resistance = R_NOMINAL;}
    float t_raw = ((T_AMBIENT + ((observer->resistance/R_NOMINAL) - 1.0f)*254.5f));
    if(t_raw > 200.0f){t_raw = 200.0f;}
    else if(t_raw < 0.0f){t_raw = 0.0f;}
    observer->temp_measured = .999f*observer->temp_measured + .001f*t_raw;
    float e = (float)observer->temperature - observer->temp_measured;
    observer->trust = (1.0f - .004f*fminf(abs(controller->dtheta_elec), 250.0f)) * (.01f*(fminf(i_sq, 100.0f)));
    observer->temperature -= observer->trust*.0001f*e;
    //printf("%.3f\n\r", e);

    if(observer->temperature > TEMP_MAX){controller->otw_flag = 1;}
    else{controller->otw_flag = 0;}
    */
}

float linearize_dtc(ControllerStruct *controller, float dtc)
{

    float duty = fmaxf(fminf(fabs(dtc), .999f), 0.0f);;
    int index = (int) (duty*127.0f);
    float val1 = controller->inverter_tab[index];
    float val2 = controller->inverter_tab[index+1];
    return val1 + (val2 - val1)*(duty*128.0f - (float)index);
}

void field_weaken(ControllerStruct *controller)
{
	/*
       /// Field Weakening ///

       controller->fw_int += .001f*(0.5f*OVERMODULATION*controller->v_bus - controller->v_ref);
       controller->fw_int = fmaxf(fminf(controller->fw_int, 0.0f), -I_FW_MAX);
       controller->i_d_ref = controller->fw_int;
       float q_max = sqrt(controller->i_max*controller->i_max - controller->i_d_ref*controller->i_d_ref);
       controller->i_q_ref = fmaxf(fminf(controller->i_q_ref, q_max), -q_max);
       //float i_cmd_mag_sq = controller->i_d_ref*controller->i_d_ref + controller->i_q_ref*controller->i_q_ref;
*/
}
void commutate(ControllerStruct *controller, ObserverStruct *observer, EncoderStruct *encoder)
{
	/* Do Field Oriented Controll and stuff */

		controller->v_bus = 1;
		controller->theta_elec = 0;

       /// Commutation Loop ///
       controller->loop_count ++;
       controller->i_b = I_SCALE*(float)(controller->adc_b_raw - controller->adc_b_offset);    // Calculate phase currents from ADC readings
       controller->i_c = I_SCALE*(float)(controller->adc_c_raw - controller->adc_c_offset);
       controller->i_a = -controller->i_b - controller->i_c;

       if((fabsf(controller->i_b) > 41.0f)|(fabsf(controller->i_c) > 41.0f)|(fabsf(controller->i_a) > 41.0f)){controller->oc_flag = 1;}

       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents

       controller->i_q_filt = 0.95f*controller->i_q_filt + 0.05f*controller->i_q;
       controller->i_d_filt = 0.95f*controller->i_d_filt + 0.05f*controller->i_d;

        controller->i_max = I_MAX*(!controller->otw_flag) + I_MAX_CONT*controller->otw_flag;

        // Temperature Controller //

        //if(observer->temperature > TEMP_MAX)
        //{
        //    float qdot_des = 1.0f*(TEMP_MAX - observer->temperature);
        //    float i_limit = sqrt((qdot_des + observer->q_out)/(R_NOMINAL*1.5f));
        //    controller->i_max = fmaxf(fminf(i_limit, I_MAX), I_MAX_CONT);
        //}
        //else{controller->i_max = I_MAX;}


        limit_norm(&controller->i_d_ref, &controller->i_q_ref, controller->i_max);

       /// PI Controller ///
       float i_d_error = controller->i_d_ref - controller->i_d;
       float i_q_error = controller->i_q_ref - controller->i_q;//  + cogging_current;

       // Calculate feed-forward voltages //
       float v_d_ff = SQRT3*(0.0f*controller->i_d_ref*R_PHASE  - controller->dtheta_elec*L_Q*controller->i_q);   //feed-forward voltages
       float v_q_ff =  SQRT3*(0.0f*controller->i_q_ref*R_PHASE +  controller->dtheta_elec*(L_D*controller->i_d + 0.0f*WB));

       // Integrate Error //
       controller->d_int += controller->k_d*controller->ki_d*i_d_error;
       controller->q_int += controller->k_q*controller->ki_q*i_q_error;

       controller->d_int = fmaxf(fminf(controller->d_int, OVERMODULATION*controller->v_bus), -OVERMODULATION*controller->v_bus);
       controller->q_int = fmaxf(fminf(controller->q_int, OVERMODULATION*controller->v_bus), -OVERMODULATION*controller->v_bus);

       //limit_norm(&controller->d_int, &controller->q_int, OVERMODULATION*controller->v_bus);
       controller->v_d = controller->k_d*i_d_error + controller->d_int;// + v_d_ff;
       controller->v_q = controller->k_q*i_q_error + controller->q_int;// + v_q_ff;
       //controller->v_q = 0.0f;
       //controller->v_d = 1.0f*controller->v_bus;
       controller->v_ref = sqrt(controller->v_d*controller->v_d + controller->v_q*controller->v_q);

       limit_norm(&controller->v_d, &controller->v_q, OVERMODULATION*controller->v_bus);       // Normalize voltage vector to lie within curcle of radius v_bus
       //float dtc = controller->v_ref/controller->v_bus;
       float scale = 0;//linearize_dtc(controller, dtc);

       //controller->v_d = scale*controller->v_d;
       //controller->v_q = scale*controller->v_q;
       //float dtc_q = controller->v_q/controller->v_bus;

       //linearize_dtc(&dtc_q);
       //controller->v_d = dtc_d*controller->v_bus;
       //controller->v_q = dtc_q*controller->v_bus;

       controller->v_d = 0;
       controller->v_q = .25f;
       controller->theta_elec = 0;

       abc(controller->theta_elec + 0.0f*DT*controller->dtheta_elec, scale*controller->v_d, scale*controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
       svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

       controller->dtc_u = .1f;
       controller->dtc_v = .2f;
       controller->dtc_w = .3f;
       set_dtc(controller);

    }


void torque_control(ControllerStruct *controller){
	/*
    float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
    //float torque_ref = -.1*(controller->p_des - controller->theta_mech);
    controller->i_q_ref = torque_ref/KT_OUT;
    controller->i_d_ref = 0.0f;
    */
    }
