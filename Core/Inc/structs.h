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

#define N_POS_SAMPLES 20		// Number of position samples to store.  should put this somewhere else...


#include <stdint.h>
#include "spi.h"
#include "gpio.h"

typedef struct{
    } GPIOStruct;

typedef struct{
    }COMStruct;

typedef struct{
	uint8_t state;
	uint8_t state_change;
}FSMStruct;

typedef struct{
    int adc1_raw, adc2_raw, adc3_raw;                       // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
    float v_bus;                                            // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float k_d, k_q, ki_d, ki_q, alpha;                      // Current loop gains, current reference filter coefficient
    float d_int, q_int;                                     // Current error integrals
    int adc1_offset, adc2_offset, adc3_offset, adc4_offset; // ADC offsets
    float i_d_ref, i_q_ref, i_d_ref_filt, i_q_ref_filt;     // Current references
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
    int mode;
    int ovp_flag;                                           // Over-voltage flag
    int oc_flag;											// Over-current flag
    float p_des, v_des, kp, kd, t_ff;                       // Desired position, velocity, gains, torque
    float v_ref, fw_int;                                    // output voltage magnitude, field-weakening integral
    int otw_flag;                                           // Over-temp warning
    float i_max;											// Maximum current
    float inverter_tab[128];								// Inverter linearization table
    } ControllerStruct;

typedef struct{
    double temperature;                                     // Estimated temperature
    float temp_measured;									// "Measured" temperature computed from resistance
    float qd_in, qd_out;									// Thermal power in and out
    float resistance;										// Motor resistance
    float k;												// Temperature observer gain
    float trust;											// Temperature observer "trust' (kind of like 1/covariance)
    float delta_t;											// Temperature rise
    }   ObserverStruct;

typedef struct{
	SPI_HandleTypeDef hspi;
	GPIO_TypeDef* cs_gpio;
	uint16_t cs_pin;
	union{
		uint8_t spi_tx_buff[2];
		uint16_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[2];
		uint16_t spi_rx_word;
	};
	float angle_singleturn, angle_multiturn[N_POS_SAMPLES], old_angle_multiturn, elec_angle, velocity, vel2, ppairs;
	int raw, count, old_count, cpr, turns;
	int m_zero, e_zero;
	int offset_lut[128];
	uint8_t first_sample;
} EncoderStruct;

typedef struct{
	SPI_HandleTypeDef hspi;
	GPIO_TypeDef* cs_gpio;
	uint16_t cs_pin;
	union{
		uint8_t spi_tx_buff[2];
		uint16_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[2];
		uint16_t spi_rx_word;
	};
	uint16_t fault;
} DRVStruct;





/* Global Structs */
extern ControllerStruct controller;
extern ObserverStruct observer;
extern COMStruct com;
extern FSMStruct state;
extern EncoderStruct comm_encoder;
extern DRVStruct drv;

#ifdef __cplusplus
}
#endif
#endif
