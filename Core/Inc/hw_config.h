#ifndef HW_CONFIG_H
#define HW_CONFIG_H


/* Timer and PWM */
#define TIM_PWM			htim1				// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_1		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_3		// Terminal W timer channel
#define INVERT_DTC		1					// PWM inverting (1) or non-inverting (0)

/* ADC */
#define ADC_CH_MAIN		hadc1				// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA		0					// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB		hadc1				// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC		hadc2				// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc3				// Bus voltage ADC channel handle.  0 = unused

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin.
#define DRV_SPI			hspi1				// DRV SPI handle
#define DRV_CS			GPIOA, GPIO_PIN_4	// DRV CS pin

/* SPI encoder */
#define ENC_SPI			hspi3				// Encoder SPI handle
#define ENC_CS			GPIOA, GPIO_PIN_15	// Encoder SPI CS pin
#define ENC_CPR			65536				// Encoder counts per revolution
#define ENC_READ_WORD	0x00				// Encoder read command

/* Misc. GPIO */
#define LED         	GPIOC, GPIO_PIN_5	// LED Pin

/* Other hardware-related constants */
#define I_SCALE 			0.02014160156f  // Amps per A/D Count
#define V_SCALE 			0.012890625f    // Bus volts per A/D Count
#define DTC_MAX 			0.94f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			.000f          	// deadtime compensation (100 ns / 25 us)
#define OVERMODULATION	 	1.15f			// Max overmodulation
#define DT					.000025f		// Loop period
#define GR					1.0f			// Gear ratio (move this later)



#endif
