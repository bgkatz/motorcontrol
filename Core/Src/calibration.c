/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */


#include "calibration.h"

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, PreferenceWriter *prefs){
/*
    ///Checks phase order, to ensure that positive Q current produces
    ///torque in the positive direction wrt the position sensor.
    printf("\n\r Checking phase ordering\n\r");
    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL;                                                             //Put all volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    int sample_counter = 0;

    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 //inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            //space vector modulation
    for(int i = 0; i<20000; i++){
        set_dtc(controller);
        HAL_Delay(100);
        }
    //ps->ZeroPosition();
    ps->Sample(DT);
    wait_us(1000);
    //float theta_start = ps->GetMechPositionFixed();                                  //get initial rotor position
    float theta_start;
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));
    printf("\n\rCurrent\n\r");
    printf("%f    %f   %f\n\r\n\r", controller->i_d, controller->i_q, current);
    /// Rotate voltage angle
    while(theta_ref < 4*PI){                                                    //rotate for 2 electrical cycles
        abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                             //inverse dq0 transform on voltages
        svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                        //space vector modulation
        wait_us(100);
        set_dtc(controller);
       ps->Sample(DT);                                                            //sample position sensor
       theta_actual = ps->GetMechPositionFixed();
       if(theta_ref==0){theta_start = theta_actual;}
       if(sample_counter > 200){
           sample_counter = 0 ;
        printf("%.4f   %.4f\n\r", theta_ref/(NPP), theta_actual);
        }
        sample_counter++;
       theta_ref += 0.001f;
        }
    float theta_end = ps->GetMechPositionFixed();
    int direction = (theta_end - theta_start)>0;
    printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
    printf("Direction:  %d\n\r", direction);
    if(direction){printf("Phasing correct\n\r");}
    else if(!direction){printf("Phasing incorrect.  Swapping phases V and W\n\r");}
    PHASE_ORDER = direction;
    */
}

void calibrate(EncoderStruct *encoder, ControllerStruct *controller, PreferenceWriter *prefs){

}
