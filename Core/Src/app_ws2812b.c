/*
 * app_ws2812b.c
 *
 *  Created on: Dec 25, 2022
 *      Author: sangmin_lee
 */

#include "app_ws2812b.h"


uint16_t g_led_data[(TOTALNUM*32)+CYCLE_RESET]={0,};//CYCLE_RESET of data = 0, not BIT_HIGH or BIT_LOW


void ws2812NumOn(uint32_t led_cnt)
{
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint16_t *)g_led_data, (TOTALNUM*32)+CYCLE_RESET);
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_2, (uint16_t *)g_led_data, (TOTALNUM*32)+CYCLE_RESET);
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, (uint16_t *)g_led_data, (TOTALNUM*32)+CYCLE_RESET);
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_4, (uint16_t *)g_led_data, (TOTALNUM*32)+CYCLE_RESET);
}


void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
	uint32_t buf[TOTALNUM]={0,};

	buf[index] = green*0x1000000 + red*0x10000 + blue*0x100;	//8bit+8bit+8bit=24bit

	for(int i=0;i<32;i++)
	{
		if(buf[index]&(1<<i)) {g_led_data[(TOTALNUM-index)*32-i] = BIT_HIGH;}
		else {g_led_data[(TOTALNUM-index)*32-i] = BIT_LOW;}
	}
}


void ws2812AllColor(uint8_t red, uint8_t green, uint8_t blue)
{
	uint32_t buf=0;

	buf = green*0x1000000 + red*0x10000 + blue*0x100;	//8bit+8bit+8bit=24bit

	for(int j=TOTALNUM;j>0;j--)
	{
		for(int i=0;i<32;i++)
		{
			if(buf&(1<<i)) {g_led_data[(j*32)-i] = BIT_HIGH;}
			else {g_led_data[(j*32)-i] = BIT_LOW;}
		}
	}
}






