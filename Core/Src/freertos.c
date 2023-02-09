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
#include "math.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

MappingPar vel_RxPDO0={{0x60ff,0,0,0},//index //target speed
						{0x03,0,0,0},//subindex //left and rigt target speed combination
						{0x20,0,0,0},//length //32bit
						0x01,//option//event timer
						500};//option_time //500

MappingPar vel_TxPDO0={{0x606C,0,0,0},//index //target speed
						{0x03,0,0,0},//subindex //left and rigt target speed combination
						{0x20,0,0,0},//length //32bit
						0x01,//option
						200};//option_time //inhibit time 10000, event time 1000 = 500ms

MappingPar vel_TxPDO1={{0x603F,0,0,0},//index //target speed
						{0x00,0,0,0},//subindex //left and rigt target speed combination
						{0x10,0,0,0},//length //16bit
						0x00,//option
						2000};//option_time //inhibit time 10000, event time 1000 = 500ms

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t PS_SIGx_Pin = 0;	//0000 4321

int16_t SteDeg;	//steering degree unit=0.01 degree


char buf[48]={	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,		//1 front right
					13, 14, 15, 16, 17, 18, 19, 20, 21, 22,	23, 24,		//2 front left
					25, 26, 27, 28, 29, 30, 31, 32,	33, 34, 35, 36,		//3 rear right
					37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48	};	//4 rear left



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


//int16_t transdata(,uint8_t RW)
//{
//
//}

int16_t rad2deg(double radian)
{
    return (int16_t)(radian*180/MATH_PI);
}

void ModeSelect(void){
	//ModeA//for linearX x linearY
	//ModeB//for linearX x angle
	//ModeC//for rotate
	//ModeD//for stop
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	osThreadFlagsSet(IRQ_PSxHandle, 1);

    if(GPIO_Pin == PS_SIG1_Pin) {
    	PS_SIGx_Pin |= 0b00000001;
    	printf("GPIO_EXTI_Callback PS_SIG1_Pin.\n");
	}

    if(GPIO_Pin == PS_SIG2_Pin) {
    	PS_SIGx_Pin |= 0b00000010;
    	printf("GPIO_EXTI_Callback PS_SIG2_Pin.\n");
    }

    if(GPIO_Pin == PS_SIG3_Pin) {
    	PS_SIGx_Pin |= 0b00000100;
    	printf("GPIO_EXTI_Callback PS_SIG3_Pin.\n");
    }

    if(GPIO_Pin == PS_SIG4_Pin) {
    	PS_SIGx_Pin |= 0b00001000;
    	printf("GPIO_EXTI_Callback PS_SIG4_Pin.\n");
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
	uint32_t CanId = 0;

	int16_t Tar_cmd_v_x = 0;
	int16_t Tar_cmd_v_y = 0;
	int16_t Tar_cmd_w = 0;

	double Tmp_cmd_v_x = 0;
	double Tmp_cmd_v_y = 0;
	double Tmp_cmd_w = 0;

	int16_t Tar_cmd_FL = 0;//Front Left
	int16_t Tar_cmd_FR = 0;//Front Right
	int16_t Tar_cmd_RL= 0;//Rear Left
	int16_t Tar_cmd_RR = 0;//Rear Right

	int16_t Tmp_cmd_FL = 0;
	int16_t Tmp_cmd_FR = 0;
	int16_t Tmp_cmd_RL= 0;
	int16_t Tmp_cmd_RR = 0;

	int16_t Real_cmd_v_x = 0;
	int16_t Real_cmd_v_y = 0;
	int16_t Real_cmd_w = 0;

	uint8_t torqueSW = 0;

	//////////////////////////////
	uint32_t lastTime = osKernelGetTickCount();

	CanInit(0,0);

	osDelay(3000);//must delay for nmt from motor driver
	PDOMapping(1, RxPDO0, vel_RxPDO0, 1);
	PDOMapping(2, RxPDO0, vel_RxPDO0, 1);

	PDOMapping(1, TxPDO0, vel_TxPDO0, 1);//event time mode 100ms
	PDOMapping(2, TxPDO0, vel_TxPDO0, 1);//event time mode
	PDOMapping(1, TxPDO1, vel_TxPDO1, 1);//inhibit mode 100ms
	PDOMapping(2, TxPDO1, vel_TxPDO1, 1);//inhibit mode


	for(int i=0;i<2;i++){
		SDOMsg(i+1,0x2010, 0x0, 0x01, 1);//Node_id, index,  subindex,  msg,  len//save eeprom
		Tor_OnOff(TORQUEON);
		SDOMsg(i+1,0x6060, 0x0, 0x03, 1);//Node_id, index,  subindex,  msg,  len//3: Profile velocity mode;
		SDOMsg(i+1,0x200f, 0x0, 0x01, 2);//Node_id, index,  subindex,  msg,  len//1e: Synchronization control
	}

	//PDOMapping(Node_id, 0x1A01, vel_TxPDO2, 2);
	Vel_PDOMsg(1, TxPDO0, 0x2, 0x1);
	Vel_PDOMsg(2, TxPDO0, 0x10, 0x20);
	//osDelay(3000);
	Vel_PDOMsg(1, TxPDO0, 0x0, 0x0);
	Vel_PDOMsg(2, TxPDO0, 0x0, 0x0);
  /* Infinite loop */
  for(;;)
  {

	lastTime += PERIOD_CANCOMM;;
	osDelayUntil(lastTime);


	//SDOMsg(1,0x1011, 0x3, 0xf1, 1);

	if(FLAG_RxCplt>0)	//real time, check stdid, extid
	{
		for(int i=0;i<8;i++){canbuf[i] = g_uCAN_Rx_Data[i];}
		FLAG_RxCplt--;
		if(g_tCan_Rx_Header.StdId>g_tCan_Rx_Header.ExtId){CanId = g_tCan_Rx_Header.StdId;}
		else {CanId = g_tCan_Rx_Header.ExtId;}

		//sendCan(1, canbuf, 8, 0);//(uint32_t ID, uint8_t data[8], uint8_t len, uint8_t ext)

		switch(CanId)//parse
		{
			case 0x3E9:
//				Tar_cmd_v_x = (int16_t)canbuf[1]<<8 | (int16_t)canbuf[0];
//				Tar_cmd_v_y = (int16_t)canbuf[3]<<8 | (int16_t)canbuf[2];
//				Tar_cmd_w = (int16_t)canbuf[5]<<8 | (int16_t)canbuf[4];
//				torqueSW = canbuf[6];
				break;

			case 0x181:
				Tmp_cmd_FL =(int16_t)canbuf[1]<<8 | (int16_t)canbuf[0];
				Tmp_cmd_FR =(int16_t)canbuf[3]<<8 | (int16_t)canbuf[2];

			case 0x182:
				Tmp_cmd_RL= (int16_t)canbuf[1]<<8 | (int16_t)canbuf[0];
				Tmp_cmd_RR =(int16_t)canbuf[3]<<8 | (int16_t)canbuf[2];
				break;

			case 2002:

				break;
		}



		g_tCan_Rx_Header.StdId=0;
		g_tCan_Rx_Header.ExtId=0;
		CanId = 0;

		for(int i=0;i<8;i++){canbuf[i]=0;}

	}

	canbuf[7] = 8;
	canbuf[6] = 7;
	canbuf[5] = 6;
	canbuf[4] = 5;
	canbuf[3] = 4;
	canbuf[2] = 3;
	canbuf[1] = 2;
	canbuf[0] = 1;
	osDelay(10);

	//sendCan(2, canbuf, 8, 0);//(uint32_t ID, uint8_t data[8], uint8_t len, uint8_t ext)
	for(int i=0;i<8;i++){canbuf[i]=0;}

	Tar_cmd_v_x=10;
	Tar_cmd_v_y=0;

	osDelay(10);
	//printf("ANGLE_RAD: %f\n", ANGLE_RAD);
	Tar_cmd_FL = CONSTANT_VEL  *  (Tar_cmd_v_x*cos(ANGLE_RAD) + Tar_cmd_v_y*sin(ANGLE_RAD));
	Tar_cmd_RR = Tar_cmd_RL = Tar_cmd_FR = Tar_cmd_FL;

	SteDeg=rad2deg(ANGLE_RAD);

	osDelay(20);
	//printf("Tar_cmd: \n");
	//printf("Tar_cmd: %d %d %d %d \n", Tar_cmd_RR,Tar_cmd_RL ,Tar_cmd_FR,Tar_cmd_FL);

	Vel_PDOMsg(1, TxPDO0, Tar_cmd_FL, -Tar_cmd_FR);
	Vel_PDOMsg(2, TxPDO0, Tar_cmd_RL, -Tar_cmd_RR);

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
	uint8_t Dir_Rot=0; //direction of rotation
//	char buf[48]={	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,		//1 front right
//					13, 14, 15, 16, 17, 18, 19, 20, 21, 22,	23, 24,		//2 front left
//					25, 26, 27, 28, 29, 30, 31, 32,	33, 34, 35, 36,		//3 rear right
//					37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48	};	//4 rear left

	osDelay(1000);

	GPIO_enableirq();
	osDelay(100);

	if(HAL_GPIO_ReadPin(GPIOA, PS_SIG1_Pin)){
		DataSetSteering(buf, 0, SERVO_CCW, RPM_2, SERVO_INIT);
		printf("PS_SIG1_Pin CCW init.\n");
	}
	else {
		DataSetSteering(buf, 0, SERVO_CW, RPM_2, SERVO_INIT);
		printf("PS_SIG1_Pin CW init.\n");
	}

	if(HAL_GPIO_ReadPin(GPIOA, PS_SIG2_Pin)){
		DataSetSteering(buf, 1, SERVO_CW, RPM_2, SERVO_INIT);
		printf("PS_SIG2_Pin CW init.\n");
	}
	else {
		DataSetSteering(buf, 1, SERVO_CCW, RPM_2, SERVO_INIT);
		printf("PS_SIG2_Pin CCW init.\n");
	}
	if(HAL_GPIO_ReadPin(GPIOA, PS_SIG3_Pin)){
		DataSetSteering(buf, 2, SERVO_CW, RPM_2, SERVO_INIT);
		printf("PS_SIG3_Pin CW init.\n");
	}
	else {
		DataSetSteering(buf, 2, SERVO_CCW, RPM_2, SERVO_INIT);
		printf("PS_SIG3_Pin CCW init.\n");
	}
	if(HAL_GPIO_ReadPin(GPIOA, PS_SIG4_Pin)){
		DataSetSteering(buf, 3, SERVO_CCW, RPM_2, SERVO_INIT);
		printf("PS_SIG4_Pin CCW init.\n");
	}
	else {
		DataSetSteering(buf, 3, SERVO_CW, RPM_2, SERVO_INIT);
		printf("PS_SIG4_Pin CW init.\n");
	}
	osDelay(1000);


	for(int i=0;i<20;i++){
		osDelay(500);
		ServoMotor_writeDMA(buf);//servo init. must done init within 500*20ms
	}
	osDelay(500);
	GPIO_disableirq();


	uint32_t lastTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	lastTime += PERIOD_STEERING;
	osDelayUntil(lastTime);

	printf("SteDeg: %d\n", SteDeg);
	if(SteDeg<0){
		SteDeg*=-1;
		Dir_Rot=SERVO_CW;
	}
	else Dir_Rot=SERVO_CCW;
	if(SteDeg>90){SteDeg=90;}
	DataSetSteering(buf, 0, Dir_Rot, SteDeg*100, SERVO_POS);
	DataSetSteering(buf, 1, Dir_Rot, SteDeg*100, SERVO_POS);
	DataSetSteering(buf, 2, Dir_Rot, SteDeg*100, SERVO_POS);
	DataSetSteering(buf, 3, Dir_Rot, SteDeg*100, SERVO_POS);
	osDelay(10);



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
		//printf("task4\n");
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
	//printf("task5\n");

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
	//char buf[48]={0,};
  /* USER CODE BEGIN StartTask06 */
	//uint32_t lastTime = osKernelGetTickCount();
	osDelay(10);//for printf();
	printf("StartTask06 PS_SIG3_Pin.%d: \n", PS_SIGx_Pin);
  /* Infinite loop */
  for(;;)
  {
		osThreadFlagsWait(1, 0, osWaitForever);

	if(PS_SIGx_Pin&1){//1ch init
		PS_SIGx_Pin=0; printf(" PS_SIG1_stop.\n");
		DataSetSteering(buf, 0, SERVO_CCW, 0, 0);
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
	}

	if(PS_SIGx_Pin&2){//2ch init
		PS_SIGx_Pin=0; printf(" PS_SIG2_stop.\n");
		DataSetSteering(buf, 1, SERVO_CCW, 0, 0);
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
	}
	if(PS_SIGx_Pin&4){//3ch init
		PS_SIGx_Pin=0; printf(" PS_SIG3_stop.\n");
		DataSetSteering(buf, 2, SERVO_CCW, 0, 0);
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
	}
	if(PS_SIGx_Pin&8){//4ch init
		PS_SIGx_Pin=0; printf(" PS_SIG4_stop.\n");
		DataSetSteering(buf, 3, SERVO_CCW, 0, 0);
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
	}


	//printf("StartTask06.\n");

  }
  /* USER CODE END StartTask06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

