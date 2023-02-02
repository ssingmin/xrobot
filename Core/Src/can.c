/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

uint32_t FLAG_RxCplt = 0;

uint8_t					g_uCAN_Rx_Data[8] = {0,};
CAN_RxHeaderTypeDef 	g_tCan_Rx_Header;

CAN_FilterTypeDef       sFilterConfig;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CanInit(uint32_t id, uint32_t mask)
{


	#if 0//example idlist mode
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;	//receive only canid of 1002, 5001
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 1002<<5;	//for receive id about 1002(pc)
    sFilterConfig.FilterIdLow = 0x0000;		//for receive id about 1002(pc)
    sFilterConfig.FilterMaskIdHigh = (5001<<3)>>16;		//for receive id about 5001(bottom board)
    sFilterConfig.FilterMaskIdLow = ((5001<<3)&0xffff)|(0x1<<2);		//for receive id about 5001(bottom board)
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;

	#else//example idmask mode
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (id<<3)>>16;
    sFilterConfig.FilterIdLow = ((id<<3)&0xffff)|(0x1<<2);
    sFilterConfig.FilterMaskIdHigh = (mask<<3)>>16;
    sFilterConfig.FilterMaskIdLow = ((mask<<3)&0xffff)|(0x1<<2);
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;
    #endif

    if (HAL_CAN_Start(&hcan1) != HAL_OK){Error_Handler();}/* Start Error */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){while(1){;}}

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
		/* Filter configuration Error */
		Error_Handler();
    }
}

void sendCan(uint32_t ID, uint8_t *buf, uint8_t len, uint8_t ext)
{

	CAN_TxHeaderTypeDef tCan_Tx_Header;

    uint32_t dwTxMailBox;
    uint32_t dwCheck;

    tCan_Tx_Header.StdId = ID;//for send id 3001
	tCan_Tx_Header.ExtId = ID;//for send id 3001
	tCan_Tx_Header.RTR = CAN_RTR_DATA;
	tCan_Tx_Header.IDE = ext ? CAN_ID_EXT : CAN_ID_STD;
	tCan_Tx_Header.DLC = len;
	tCan_Tx_Header.TransmitGlobalTime = DISABLE;

    dwTxMailBox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);	//resolve the error situation

    if(dwTxMailBox == 0){}
    else
    {
        dwCheck = HAL_CAN_AddTxMessage(&hcan1, &tCan_Tx_Header, buf, &dwTxMailBox);
        if(dwCheck != HAL_OK){while(1){;}}
    }
    osDelay(1);
}

void SDOMsg(uint8_t Node_id,uint16_t index, uint8_t subindex, uint32_t msg, uint8_t len)
{
	uint8_t buf[8]={0,};

	switch (len) {
		case 1:
			buf[0]=0x2f;	break;	//1byte
		case 2:
			buf[0]=0x2b;	break;	//2byte
		case 3:
			buf[0]=0x27;	break;	//3byte
		case 4:
			buf[0]=0x23;	break;	//4byte
	}

	memcpy(buf+1,&index,2);	//index
	buf[3]=subindex;		//subindex
	memcpy(buf+4,&msg,len);	//data

	sendCan(0x600+Node_id,buf,8,0);
}

void NMT_Mode(uint8_t command, uint8_t Node_id)// command 1= pre-operation, 2=operation
{
	uint8_t buf[8]={0,};


	if(command == 1){buf[0]=0x80;}//enter nmt pre-operational command
	else{buf[0]=0x01;}//enter nmt operational command for PDO operation
	buf[1]=Node_id;//node id

	sendCan(0, buf, 8, 0);
}


void PDOMapping(uint8_t Node_id, uint16_t PDO_index, MappingPar Param, uint8_t Num_entry)//entry rr
{
	uint32_t tmp=0;

	osDelay(1);
	//printf("Num_entry!!!: %02X\n", Num_entry);

	NMT_Mode(1, 1);//pre-operation mode

	SDOMsg(Node_id, PDO_index, 0, 0, 1);//clear rpdo0 mapping
	for(int i=0;i<Num_entry;i++) {//clear rpdo0 mapping, 0x60ff(index) 03(subindex) 20(length)
		tmp=(0x10000*Param.index[i])+(0x100* Param.subindex[i])+(Param.length[i]);
		SDOMsg(Node_id, PDO_index, i+1, tmp, 4);
	}
	SDOMsg(Node_id, PDO_index-0x200, 1, 0x200+Node_id, 4);//cob-id??
	SDOMsg(Node_id, PDO_index-0x200, 2, 0xff, 1);//transmission type, fix asynchronous with 0xff
	SDOMsg(Node_id, PDO_index-0x200, 3+(Param.option*2), Param.option_time, 4);//not necessary 3= inhibit mode, 5=event timer mode
	SDOMsg(Node_id, PDO_index, 0, 0x01, 1);//clear rpdo0 mapping

	NMT_Mode(2, 1);//operation mode
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &g_tCan_Rx_Header, g_uCAN_Rx_Data) != HAL_OK){while(1){;}}
	FLAG_RxCplt++;
}


/* USER CODE END 1 */
