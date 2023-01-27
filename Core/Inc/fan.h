/*
 * fan.h
 *
 *  Created on: May 17, 2022
 *      Author: sangmin_lee
 */
 #ifndef __FAN_H__
 #define __FAN_H__

#include "main.h"
#include "stm32f4xx_hal_tim_ex.h"

extern TIM_HandleTypeDef htim1;

void fanInit(void);
void fanOn(uint8_t duty);

#endif 
