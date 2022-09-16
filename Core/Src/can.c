/*
 * can.c
 *
 *  Created on: May 17, 2022
 *      Author: sangmin_lee
 */

#include <can.h>


uint32_t FLAG_RxCplt = 0;

uint8_t					g_uCAN_Rx_Data[8] = {0,};
CAN_RxHeaderTypeDef 	g_tCan_Rx_Header;

CAN_FilterTypeDef       sFilterConfig;

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
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get RX message */

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &g_tCan_Rx_Header, g_uCAN_Rx_Data) != HAL_OK){while(1){;}}
	FLAG_RxCplt++;

}








