/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "fan.h"
#include "can.h"
#include "servo_motor.h"
#include "definition.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canTask */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for UartComm */
osThreadId_t UartCommHandle;
const osThreadAttr_t UartComm_attributes = {
  .name = "UartComm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NP_LED */
osThreadId_t NP_LEDHandle;
const osThreadAttr_t NP_LED_attributes = {
  .name = "NP_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fancntl */
osThreadId_t fancntlHandle;
const osThreadAttr_t fancntl_attributes = {
  .name = "fancntl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IRQ_PSx */
osThreadId_t IRQ_PSxHandle;
const osThreadAttr_t IRQ_PSx_attributes = {
  .name = "IRQ_PSx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for PSx_SIG_BinSem */
osSemaphoreId_t PSx_SIG_BinSemHandle;
const osSemaphoreAttr_t PSx_SIG_BinSem_attributes = {
  .name = "PSx_SIG_BinSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	osThreadFlagsSet(IRQ_PSxHandle, 1);

    if(GPIO_Pin == PS_SIG1_Pin) {
    	//osSemaphoreRelease(PSx_SIG_BinSemHandle);
    	printf("GPIO_EXTI_Callback PS_SIG1_Pin.\n");
	}

    if(GPIO_Pin == PS_SIG2_Pin) {	//for touch sensor
    	//printf("GPIO_EXTI_Callback PS_SIG2_Pin.\n");
    }

    if(GPIO_Pin == PS_SIG3_Pin) {
    	//printf("GPIO_EXTI_Callback PS_SIG3_Pin.\n");
    }

    if(GPIO_Pin == PS_SIG4_Pin) {
    	//printf("GPIO_EXTI_Callback PS_SIG4_Pin.\n");
    }
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of PSx_SIG_BinSem */
  PSx_SIG_BinSemHandle = osSemaphoreNew(1, 1, &PSx_SIG_BinSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of canTask */
  canTaskHandle = osThreadNew(StartTask02, NULL, &canTask_attributes);

  /* creation of UartComm */
  UartCommHandle = osThreadNew(StartTask03, NULL, &UartComm_attributes);

  /* creation of NP_LED */
  NP_LEDHandle = osThreadNew(StartTask04, NULL, &NP_LED_attributes);

  /* creation of fancntl */
  fancntlHandle = osThreadNew(StartTask05, NULL, &fancntl_attributes);

  /* creation of IRQ_PSx */
  IRQ_PSxHandle = osThreadNew(StartTask06, NULL, &IRQ_PSx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  This indicates overall operational state. The blue led blinks at 1Hz.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	//StartTask01 is related gpio toggle for state check //
	uint32_t lastTime = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	lastTime += PERIOD_STATUS_LED;
	osDelayUntil(lastTime);

	HAL_GPIO_TogglePin(testled_GPIO_Port, testled_Pin);
	//osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief This task parses the CAN communication. and send can message.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	//StartTask02 is related CAN communication. //
	uint8_t canbuf[8]={1, 2, 3, 4, 5, 6, 7, 8};
	//MappingPar tmp={{0x1234,0x2341,0x3412,0x4123}, {0x55,0x56,0x65,0x66}, {0x77,0x78,0x87,0x88}, 0x9a, 0xbcde};//example

	uint32_t lastTime = osKernelGetTickCount();

	CanInit(0,0);

	PDOMapping(1, 0x1600, tmp, 4);

  /* Infinite loop */
  for(;;)
  {


	lastTime += PERIOD_CANCOMM;;
	osDelayUntil(lastTime);

	SDOMsg(1,0x1011, 0x3, 0xf1, 1);
//	SDOMsg(2,0x2022, 0x33, 0xf1f2, 2);
//	SDOMsg(3,0x3033, 0x13, 0xf1f2f3, 3);
//	SDOMsg(4,0x4044, 0x14, 0xf1f2f3f4, 4);


  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief StartTask03 is related 485 task for nuri motor. must change uart port.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	char buf[48]={	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,		//1 front right
					13, 14, 15, 16, 17, 18, 19, 20, 21, 22,	23, 24,		//2 front left
					25, 26, 27, 28, 29, 30, 31, 32,	33, 34, 35, 36,		//3 rear right
					37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48	};	//4 rear left

	uint32_t lastTime = osKernelGetTickCount();


  /* Infinite loop */
  for(;;)
  {
	lastTime += PERIOD_STEERING;
	osDelayUntil(lastTime);
	DataSetSteering(buf, 3, 0, 0, 0);

	ServoMotor_writeDMA(buf);//use osdelay(6)*2ea

  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the NP_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	//StartTask04 is related ws2812b//
	uint32_t lastTime = osKernelGetTickCount();

	static int temp = 0;
	////////////////////////////////


  /* Infinite loop */
  for(;;)
  {
		lastTime += PERIOD_NP_LED;
		osDelayUntil(lastTime);


		temp++;
		switch (temp) {
			case 1:
				printf("case1\n");
				ws2812SetColor(0,0,0,1);//index, r, g, b
				ws2812SetColor(1,0,1,0);//index, r, g, b
				ws2812SetColor(2,1,0,0);//index, r, g, b
				ws2812SetColor(3,0,0,1);//index, r, g, b
				ws2812SetColor(4,0,1,0);//index, r, g, b
				ws2812SetColor(5,1,0,0);//index, r, g, b
				ws2812SetColor(6,0,0,1);//index, r, g, b
				ws2812SetColor(7,0,1,0);//index, r, g, b
				break;
			case 2:
				printf("case2\n");
				ws2812SetColor(7,0,0,1);//index, r, g, b
				ws2812SetColor(0,0,1,0);//index, r, g, b
				ws2812SetColor(1,1,0,0);//index, r, g, b
				ws2812SetColor(2,0,0,1);//index, r, g, b
				ws2812SetColor(3,0,1,0);//index, r, g, b
				ws2812SetColor(4,1,0,0);//index, r, g, b
				ws2812SetColor(5,0,0,1);//index, r, g, b
				ws2812SetColor(6,0,1,0);//index, r, g, b
				break;
			case 3:
				printf("case3\n");
				ws2812SetColor(6,0,0,1);//index, r, g, b
				ws2812SetColor(7,0,1,0);//index, r, g, b
				ws2812SetColor(0,1,0,0);//index, r, g, b
				ws2812SetColor(1,0,0,1);//index, r, g, b
				ws2812SetColor(2,0,1,0);//index, r, g, b
				ws2812SetColor(3,1,0,0);//index, r, g, b
				ws2812SetColor(4,0,0,1);//index, r, g, b
				ws2812SetColor(5,0,1,0);//index, r, g, b
				break;
			case 4:
				printf("case4\n");
				ws2812SetColor(5,0,0,1);//index, r, g, b
				ws2812SetColor(6,0,1,0);//index, r, g, b
				ws2812SetColor(7,1,0,0);//index, r, g, b
				ws2812SetColor(0,0,0,1);//index, r, g, b
				ws2812SetColor(1,0,1,0);//index, r, g, b
				ws2812SetColor(2,1,0,0);//index, r, g, b
				ws2812SetColor(3,0,0,1);//index, r, g, b
				ws2812SetColor(4,0,1,0);//index, r, g, b
				break;
			case 5:
				printf("case5\n");
				ws2812SetColor(4,0,0,1);//index, r, g, b
				ws2812SetColor(5,0,1,0);//index, r, g, b
				ws2812SetColor(6,1,0,0);//index, r, g, b
				ws2812SetColor(7,0,0,1);//index, r, g, b
				ws2812SetColor(0,0,1,0);//index, r, g, b
				ws2812SetColor(1,1,0,0);//index, r, g, b
				ws2812SetColor(2,0,0,1);//index, r, g, b
				ws2812SetColor(3,0,1,0);//index, r, g, b
				ws2812SetColor(8,0,0,1);//index, r, g, b
				ws2812SetColor(9,0,1,0);//index, r, g, b

				break;
			case 6:
				printf("case6\n");
				ws2812SetColor(3,0,0,1);//index, r, g, b
				ws2812SetColor(4,0,1,0);//index, r, g, b
				ws2812SetColor(5,1,0,0);//index, r, g, b
				ws2812SetColor(6,0,0,1);//index, r, g, b
				ws2812SetColor(7,0,1,0);//index, r, g, b
				ws2812SetColor(0,1,0,0);//index, r, g, b
				ws2812SetColor(1,0,0,1);//index, r, g, b
				ws2812SetColor(2,0,1,0);//index, r, g, b
				ws2812SetColor(8,0,1,1);//index, r, g, b
				ws2812SetColor(9,1,1,0);//index, r, g, b
				break;
			case 7:
				printf("case7\n");
				ws2812SetColor(2,0,0,1);//index, r, g, b
				ws2812SetColor(3,0,1,0);//index, r, g, b
				ws2812SetColor(4,1,0,0);//index, r, g, b
				ws2812SetColor(5,0,0,1);//index, r, g, b
				ws2812SetColor(6,0,1,0);//index, r, g, b
				ws2812SetColor(7,1,0,0);//index, r, g, b
				ws2812SetColor(0,0,0,1);//index, r, g, b
				ws2812SetColor(1,0,1,0);//index, r, g, b
				break;
			case 8:
				printf("case8\n");
				ws2812SetColor(1,0,0,1);//index, r, g, b
				ws2812SetColor(2,0,1,0);//index, r, g, b
				ws2812SetColor(3,1,0,0);//index, r, g, b
				ws2812SetColor(4,0,0,1);//index, r, g, b
				ws2812SetColor(5,0,1,0);//index, r, g, b
				ws2812SetColor(6,1,0,0);//index, r, g, b
				ws2812SetColor(7,0,0,1);//index, r, g, b
				ws2812SetColor(0,0,1,0);//index, r, g, b
				ws2812SetColor(8,1,1,1);//index, r, g, b
				ws2812SetColor(9,1,1,1);//index, r, g, b
				temp=1;
				break;
		}

		ws2812AllColor(0,0,0);//r, g, b
		ws2812NumOn(NUM_NPLED);
		printf("task4\n");
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the fancntl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	uint32_t lastTime = osKernelGetTickCount();

	fanInit();

  /* Infinite loop */
  for(;;)
  {
	lastTime += PERIOD_FAN;
	osDelayUntil(lastTime);
	fanOn(30);
	//htim1.Instance->CCR1 = 50;
	printf("task5\n");

  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the IRQ_PSx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
	//uint32_t lastTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
		//lastTime += PERIOD_IRQ_PSx;
		//osDelayUntil(lastTime);
	//	  if(PSx_SIG_BinSemHandle != NULL)
	//	  	  {
	//	  		  if(osSemaphoreAcquire(PSx_SIG_BinSemHandle, 0) == osOK)
	//	  		  {
	//	  			printf("StartTask06 PS_SIG1_Pin.\n");
	//	  		  }
	//	  	  }
		  osThreadFlagsWait(1, 0, osWaitForever);
		  printf("StartTask06 PS_SIG1_Pin.\n");


  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

