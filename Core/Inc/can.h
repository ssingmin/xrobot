/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

extern uint32_t FLAG_RxCplt;
extern int8_t g_uCAN_Rx_Data[6][8];
extern CAN_RxHeaderTypeDef g_tCan_Rx_Header[6];

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define PRE_OPERATION 1
#define OPERATION 2

#define  RxPDO0 0x1600
#define  TxPDO0 0x1A00
#define  TxPDO1 0x1A01

#define  TORQUEON 1
#define  TORQUEOFF 0

///////////////////////////////////////
//0011 1110 1001	//0x3E9
//0010 1000 0001	//0x281
//0010 1000 0010	//0x282
//0010 1000 0010	//0x282
//
//1110 1001 0100	maskid//111010010100//0xE94
//0010 1000 0000	filterid//0010100000000//0x280
//010 00110 0000	//0x460
//111 11010 0001	//0xFA1

#define FILTERID 0x80
#define MASKID 0xC94
#define STDID 0x0
#define EXTID 0x1
///////////////////////////////////////

typedef struct _MappingPar {
	uint16_t index[4];
	uint8_t subindex[4];
	uint8_t length[4];
	uint8_t option;//0=inhibit time, 1=event timer
	uint16_t option_time;
} MappingPar;


/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CanInit(uint32_t id, uint32_t mask, uint8_t EXT_Select);
void CanInit2(uint32_t id, uint32_t mask, uint8_t EXT_Select);
void sendCan(uint32_t ID, uint8_t *buf, uint8_t len, uint8_t ext);
void SDOMsg(uint8_t Node_id,uint16_t index, uint8_t subindex, uint32_t msg, uint8_t len);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);
void CAN_disableirq(void);
void CAN_enableirq(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

