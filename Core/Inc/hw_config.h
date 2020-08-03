#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Timer and PWM */
#define TIM_PWM			htim1				// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_1		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_3		// Terminal W timer channel
#define INVERT_DTC		0					// PWM inverting (1) or non-inverting (0)

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin.
#define DRV_SPI			hspi1				// DRV SPI handle
#define DRV_CS			GPIOA, GPIO_PIN_4	// DRV CS pin

/* Misc. GPIO */
#define LED         	GPIOC, GPIO_PIN_5	// LED Pin

/* Hardware-related constants */
#define I_SCALE 0.02014160156f  			// Amps per A/D Count
#define V_SCALE 0.012890625f    			// Bus volts per A/D Count
#define DTC_MAX 0.94f          				// Max phase duty cycle
#define DTC_MIN 0.0f          				// Min phase duty cycle
#define DTC_COMP .000f          			// deadtime compensation (100 ns / 25 us)



#endif
