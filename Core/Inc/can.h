/*
 * can.h
 *
 *  Created on: May 17, 2022
 *      Author: sangmin_lee
 */

#ifndef __CAN_H_
#define __CAN_H_

#include "main.h"
//#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;

void CanInit(uint32_t id, uint32_t mask);
void sendCan(uint32_t ID, uint8_t *buf, uint8_t len, uint8_t ext);

     
#endif /* __CAN_H_ */
