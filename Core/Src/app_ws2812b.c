/*
 * app_ws2812b.c
 *
 *  Created on: Dec 25, 2022
 *      Author: sangmin_lee
 */

#include "app_ws2812b.h"


uint16_t g_led_data[(TOTALNUM*24)+CYCLE_RESET]={0,};//CYCLE_RESET of data = 0, not BIT_HIGH or BIT_LOW


void ws2812Init(uint32_t led_cnt)
{
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint16_t *)g_led_data, (TOTALNUM*24)+CYCLE_RESET);
}


void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
	uint32_t buf[TOTALNUM];

	buf[index] = green*0x10000 + red*0x100 + blue;	//8bit+8bit+8bit=24bit

	for(int i=0;i<24;i++)
	{
		if(buf[index]&(1<<i)) {g_led_data[(TOTALNUM-index)*24-i] = BIT_HIGH;}
		else {g_led_data[(TOTALNUM-index)*24-i] = BIT_LOW;}
	}
}


void ws2812AllColor(uint8_t red, uint8_t green, uint8_t blue)
{
	uint32_t buf;

	buf = green*0x10000 + red*0x100 + blue;	//8bit+8bit+8bit=24bit

	for(int j=TOTALNUM;j>0;j--)
	{
		for(int i=0;i<24;i++)
		{
			if(buf&(1<<i)) {g_led_data[(j*24)-i] = BIT_HIGH;}
			else {g_led_data[(j*24)-i] = BIT_LOW;}
		}
	}
}






