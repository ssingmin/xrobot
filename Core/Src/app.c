/*
 * app.c
 *
 *  Created on: Nov 23, 2022
 *      Author: sangmin_lee
 */

#include "app.h"
#include "can.h"

int app(void)
{
	CanInit(0,0);
	while(1)
	{
		HAL_GPIO_TogglePin(testled_GPIO_Port, testled_Pin);
		uint8_t canbuf[8]={1, 2, 3, 4, 5, 6, 7, 8};

		//for(int i=0;i<8;i++){canbuf[i]=0;}
		sendCan(0, canbuf, 8, 0);//(uint32_t ID, uint8_t data[8], uint8_t len, uint8_t ext)
		printf("test\n");
		HAL_Delay(500);
	}
}
