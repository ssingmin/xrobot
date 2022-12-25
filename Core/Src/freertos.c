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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartComm */
osThreadId_t UartCommHandle;
const osThreadAttr_t UartComm_attributes = {
  .name = "UartComm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for NP_LED */
osThreadId_t NP_LEDHandle;
const osThreadAttr_t NP_LED_attributes = {
  .name = "NP_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint32_t lastTime = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	lastTime += 500U;
	osDelayUntil(lastTime);
	HAL_GPIO_TogglePin(testled_GPIO_Port, testled_Pin);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint32_t lastTime = osKernelGetTickCount();

	CanInit(0,0);
  /* Infinite loop */
  for(;;)
  {
	  uint8_t canbuf[8]={1, 2, 3, 4, 5, 6, 7, 8};

	lastTime += 500U;;
	osDelayUntil(lastTime);

	//for(int i=0;i<8;i++){canbuf[i]=0;}
	sendCan(0, canbuf, 8, 0);//(uint32_t ID, uint8_t data[8], uint8_t len, uint8_t ext
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the UartComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint32_t lastTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)//485 task for nuri motor must change uart port
  {
	lastTime += 500U;
	osDelayUntil(lastTime);
	  //HAL_UART_Transmit(&huart3, "hihi\n", sizeof("hihi\n"), 10);
	printf("hihihi\n");
    //osDelayUntil(&lastTime, 1000);

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
	uint32_t lastTime = osKernelGetTickCount();

	uint16_t value[60+24]={0,};

	////////////////////////////////
	HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint16_t *)value, 60+24);
	//HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

  /* Infinite loop */
  for(;;)
  {
		lastTime += 500U;
		osDelayUntil(lastTime);

		value[0]=30;
		value[1]=30;
		value[2]=30;
		value[3]=30;
		value[4]=30;
		value[5]=30;
		value[6]=30;
		value[7]=30;
		value[8]=30;
		value[9]=30;
		value[10]=30;
		value[11]=30;
		value[12]=30;
		value[13]=30;
		value[14]=30;
		value[15]=30;
		value[16]=59;
		value[17]=59;
		value[18]=59;
		value[19]=59;
		value[20]=59;
		value[21]=59;
		value[22]=59;
		value[23]=59;


//		value+=50;
//		if(value>100){value=0;}
		//htim8.Instance->CCR1 = value;

		printf("task4\n");
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

