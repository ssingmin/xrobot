/*
 * fan.c
 *
 *  Created on: May 17, 2022
 *      Author: sangmin_lee
 */

#include <fan.h>


void fanInit(void)
{
    //HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);//stm32f103
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void fanOn(uint8_t duty)
{
	if(duty>=100){duty = 100;}

	if(duty==0){htim1.Instance->CCR1 = 0;}
	else {htim1.Instance->CCR1 = duty;}//write gogo
}

