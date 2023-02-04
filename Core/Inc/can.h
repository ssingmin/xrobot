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

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define PRE_OPERATION 1
#define OPERATION 2

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
void CanInit(uint32_t id, uint32_t mask);
void sendCan(uint32_t ID, uint8_t *buf, uint8_t len, uint8_t ext);
void SDOMsg(uint8_t Node_id,uint16_t index, uint8_t subindex, uint32_t msg, uint8_t len);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

