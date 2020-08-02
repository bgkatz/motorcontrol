/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_flash.h"
#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "user_config.h"
#include "fsm.h"
#include "drv8323.h"
#include "math_ops.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define VERSION_NUM 2.0f


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

int count = 0;

/* Structs for control, etc */

ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
FSMStruct state;
EncoderStruct comm_encoder;
DRVStruct drv;


uint8_t Serial2RxBuffer[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */



  preference_writer_init(&prefs, 6);
  preference_writer_load(prefs);
  /* Sanitize configs in case flash is empty*/
  if(E_ZERO==-1){E_ZERO = 0;}
  if(M_ZERO==-1){M_ZERO = 0;}
  if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
  if(isnan(I_MAX) || I_MAX ==-1){I_MAX=40;}
  if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=12;}
  if(CAN_ID==-1){CAN_ID = 1;}
  if(CAN_MASTER==-1){CAN_MASTER = 0;}
  if(CAN_TIMEOUT==-1){CAN_TIMEOUT = 1000;}
  if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
  if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 125.0f;}
  if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
  if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 21.0f;}

  printf("\r\nVersion Number: %.2f\r\n", VERSION_NUM);

  /*
  printf("Hello\r\n");
  printf("Floats: %f  %f  %f\r\n", __float_reg[0], __float_reg[1], __float_reg[2]);
  printf("Ints:   %d  %d  %d\r\n", __int_reg[0], __int_reg[1], __int_reg[2]);

  __float_reg[0] = 0.0f;
  __float_reg[1] = 1.1f;
  __float_reg[2] = 2.2f;
  __int_reg[0] = 0;
  __int_reg[1] = 1;
  __int_reg[2] = 2;

  //if(!preference_writer_ready(prefs)) preference_writer_open(&prefs);
  preference_writer_open(&prefs);
  preference_writer_flush(&prefs);
  preference_writer_close(&prefs);

*/



  /* commutation encoder setup */
  comm_encoder.hspi = hspi3;				// SPI handle
  comm_encoder.cs_gpio = GPIOA;				// SPI CS GPIO
  comm_encoder.cs_pin = GPIO_PIN_15;		// SPI CS pin number
  comm_encoder.cpr = 65536;					// encoder counts per revolution
  comm_encoder.m_zero = M_ZERO;
  comm_encoder.e_zero = E_ZERO;
  comm_encoder.ppairs = PPAIRS;
  ps_warmup(&comm_encoder, 100);
  memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));	// Copy the linearization lookup table
  memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));

  /* DRV8323 setup */
  drv.hspi = hspi1;				// SPI handle
  drv.cs_gpio = GPIOA;			// SPI CS GPIO
  drv.cs_pin = GPIO_PIN_4;		// SPI CS pin number

  HAL_GPIO_WritePin(drv.cs_gpio, drv.cs_pin, GPIO_PIN_SET ); 	// CS high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET );
  drv_calibrate(drv);
  drv_write_DCR(drv, 0x0, DIS_GDF_DIS, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1, SEN_LVL_1_0);
  //zero_current(&controller.adc1_offset, &controller.adc2_offset);
  HAL_Delay(1);
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x1, 0x0, 0x0, 0x0, SEN_LVL_1_0);
  drv_write_OCPCR(drv, TRETRY_50US, DEADTIME_50NS, OCP_NONE, OCP_DEG_8US, VDS_LVL_1_88);
  drv_disable_gd(drv);
  drv_enable_gd(drv);


  /* Turn on PWM, interrupts */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


  HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim1);


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(100);
	  drv_print_faults(drv);
	  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET );
	  //HAL_Delay(100);
	  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET );
	  //printf("hello\r\n");
	  //printf("%f  %f  %f\r\n", comm_encoder.angle_multiturn[0], comm_encoder.velocity, comm_encoder.vel2);
	  //ps_sample(&comm_encoder, .000025f);
	  //for(int i = 0; i<N_POS_SAMPLES; i++){ printf(" %.2f", comm_encoder.angle_multiturn[i]);}
	  //printf("\r\n");

	  //printf("Main Loop Serial: %d", Serial2RxBuffer[1]);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
