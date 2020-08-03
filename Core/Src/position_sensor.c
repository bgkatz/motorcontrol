/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
		encoder->spi_tx_word = 0x0000;
		HAL_GPIO_WritePin(encoder->cs_gpio, encoder->cs_pin, GPIO_PIN_RESET ); 	// CS low
		HAL_SPI_TransmitReceive(&encoder->hspi, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
		while( encoder->hspi.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(encoder->cs_gpio, encoder->cs_pin, GPIO_PIN_SET ); 	// CS high
	}
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->old_count = encoder->count;
	for(int i = 1; i<N_POS_SAMPLES; i++){encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];}
	//memmove(&encoder->angle_multiturn[1], &encoder->angle_multiturn[0], (N_POS_SAMPLES-1)*sizeof(float)); // this is much slower for some reason

	/* SPI read/write */
	encoder->spi_tx_word = 0x0000;
	HAL_GPIO_WritePin(encoder->cs_gpio, encoder->cs_pin, GPIO_PIN_RESET ); 	// CS low
	HAL_SPI_TransmitReceive(&encoder->hspi, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
	while( encoder->hspi.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(encoder->cs_gpio, encoder->cs_pin, GPIO_PIN_SET ); 	// CS high
	encoder->raw = encoder ->spi_rx_word;

	/* Linearization */
	int off_1 = encoder->offset_lut[encoder->raw>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;

	/* Real angles in radians */
	encoder->angle_singleturn = 2.0f*PI_F*fmodf(((float)(encoder->count-encoder->m_zero))/((float)encoder->cpr), 1.0f);
	encoder->elec_angle = 2.0f*PI_F*fmodf((encoder->ppairs*(float)(encoder->count-encoder->e_zero))/((float)encoder->cpr), 1.0f);

	/* Rollover */
	int count_diff = encoder->count - encoder->old_count;
	if(count_diff > encoder->cpr>>1){encoder->turns--;}
	else if(count_diff < -encoder->cpr>>1){encoder->turns++;}

	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + 2.0f*PI_F*(float)encoder->turns;

	/** Velocity */
	/*
	float vel_sum;
	for(int i = 0; i<N_POS_SAMPLES/2; i++){
		vel_sum += (encoder->angle_multiturn[i] - encoder->angle_multiturn[i+(N_POS_SAMPLES/2)]);
	}

	float s2 = N_POS_SAMPLES*encoder->angle_multiturn[0];

	encoder->velocity = vel_sum/(dt*(float)(N_POS_SAMPLES/2)*(float)(N_POS_SAMPLES/2));
*/
/*
	float m = (float)N_POS_SAMPLES;
	float w = 1.0f/m;
	float q = 12.0f/(m*m*m - m);
	float c1 = 0.0f;
	float ibar = (m - 1.0f)/2.0f;
	for(int i = 0; i<N_POS_SAMPLES; i++){
		c1 += encoder->angle_multiturn[i]*q*(i - ibar);
	}
	encoder->vel2 = -c1/dt;
	*/

	encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)N_POS_SAMPLES);
}
