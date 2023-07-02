/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
//#define P_MIN -12.5f
//#define P_MAX 12.5f
//#define V_MIN -65.0f
//#define V_MAX 65.0f
#define KP_MIN 0.0f
//#define KP_MAX 500.0f
#define KD_MIN 0.0f
//#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define VB_MIN 0.0f
#define VB_MAX 40.0f
#define SENSE_BUFFER 0.0f

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t id;
	uint8_t data[7];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

void can_rx_init(CANRxMessage *msg);
void can_tx_init(CANTxMessage *msg);
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb);
void unpack_cmd(CANRxMessage msg, float *commands);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
