/*
 * app_ws2812b.h
 *
 *  Created on: Dec 25, 2022
 *      Author: sangmin_lee
 */

#ifndef INC_APP_WS2812B_H_
#define INC_APP_WS2812B_H_

#include <stdint.h>
#include "main.h"

extern TIM_HandleTypeDef htim8;

#define BIT_PERIOD      (104)
#define BIT_HIGH        (67)
#define BIT_LOW         (33)
#define CYCLE_RESET		(60)

#define TOTALNUM		(8)


void ws2812Init(uint32_t led_cnt);
void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue);
void ws2812AllColor(uint8_t red, uint8_t green, uint8_t blue);


#endif /* INC_APP_WS2812B_H_ */