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
#define VERSION_MAJOR 2
#define VERSION_MINOR 3

#define TAR_RPM	10
#define MS_PER_DEG	(1000/(6*TAR_RPM))
//#define MS_PER_DEG	11.11
#define RES_SM	100	//SM= STEERING MOTOR
#define LIMIT_MODE_C 300//300=50deg, 460=30deg, 280=55deg

extern uint8_t tmp_rx[4][SERVO_RXBUFLEN];
extern int flag_rx;

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
//						200};//option_time //inhibit time 10000, event time 1000 = 500ms

MappingPar vel_TxPDO1={{0x603F,0,0,0},//index //error code
						{0x00,0,0,0},//subindex //left and rigt target speed combination
						{0x10,0,0,0},//length //16bit
						0x00,//option
						2000};//option_time //inhibit time 10000, event time 1000 = 500ms

uint32_t STM_FT_ID[4][2] = {	{312,135},	//id1 cw = 3.12, ccw = 1.35 degree
							{0,400},	//id2 cw = 0.0, ccw = 4.0 degree
							{0,500},	//id3 cw = 0.0, ccw = 5.0 degree
							{400,100}};	//id4 cw = 4.0, ccw = 1.0 degree


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int8_t candbg[8]={0,};
uint8_t PS_SIGx_Pin = 0;	//0000 4321

int16_t SteDeg[4] = {0,};	//steering degree unit=0.01 degree

uint8_t ModeABCD = 4;//4 is stop mode
//uint8_t ModeABCD = 1;//4 is stop mode
uint8_t Pre_ModeABCD = 0;
uint8_t STinitdone = 0;
uint32_t Stop_flag = 0;
uint32_t Pre_Stop_flag = 0;
//uint8_t EndModeD = 0;
uint8_t timerflag = 1;
uint8_t EndMode = 1;

uint8_t buf[48]={	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,		//1 front right
					13, 14, 15, 16, 17, 18, 19, 20, 21, 22,	23, 24,		//2 front left
					25, 26, 27, 28, 29, 30, 31, 32,	33, 34, 35, 36,		//3 rear right
					37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48	};	//4 rear left

int32_t SAngle[4] = {0,};

double real_angle_c;
double real_angle_i;
double real_angle_o;

double angle_rad_c;
double angle_rad_i;
double angle_rad_o;

//int16_t Tar_cmd_v_x = 0;
//int16_t Tar_cmd_v_y = 0;
//int16_t Tar_cmd_w = 0;

double Tar_cmd_v_x = 0;
double Tar_cmd_v_i = 0;
double Tar_cmd_v_o = 0;
double Tar_cmd_v_y = 0;
double Tar_cmd_w = 0;


int16_t temp_x = 0;
int16_t temp_y = 0;
int16_t temp_w = 0;
int8_t state_stop = 0;


double Real_cmd_v_x = 0;
double Real_cmd_v_y = 0;
double Real_cmd_w = 0;

int16_t Tmp_cmd_FL = 0;
int16_t Tmp_cmd_FR = 0;
int16_t Tmp_cmd_RL= 0;
int16_t Tmp_cmd_RR = 0;

int16_t Tar_cmd_FL = 0;//Front Left
int16_t Tar_cmd_FR = 0;//Front Right
int16_t Tar_cmd_RL= 0;//Rear Left
int16_t Tar_cmd_RR = 0;//Rear Right

int8_t canbuf[8]={0,};
int8_t sendcanbuf[8]={0,};

uint32_t CanId = 0;

uint8_t EndInit = 0;

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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for UartComm */
osThreadId_t UartCommHandle;
const osThreadAttr_t UartComm_attributes = {
  .name = "UartComm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
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
/* Definitions for steeringtask */
osThreadId_t steeringtaskHandle;
const osThreadAttr_t steeringtask_attributes = {
  .name = "steeringtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for VelStopTimer */
osTimerId_t VelStopTimerHandle;
const osTimerAttr_t VelStopTimer_attributes = {
  .name = "VelStopTimer"
};
/* Definitions for EndModeDTimer */
osTimerId_t EndModeDTimerHandle;
const osTimerAttr_t EndModeDTimer_attributes = {
  .name = "EndModeDTimer"
};
/* Definitions for SendCanTimer */
osTimerId_t SendCanTimerHandle;
const osTimerAttr_t SendCanTimer_attributes = {
  .name = "SendCanTimer"
};
/* Definitions for canmsg */
osMutexId_t canmsgHandle;
const osMutexAttr_t canmsg_attributes = {
  .name = "canmsg"
};
/* Definitions for Degmsg */
osMutexId_t DegmsgHandle;
const osMutexAttr_t Degmsg_attributes = {
  .name = "Degmsg"
};
/* Definitions for Stop_flag */
osMutexId_t Stop_flagHandle;
const osMutexAttr_t Stop_flag_attributes = {
  .name = "Stop_flag"
};
/* Definitions for PSx_SIG_BinSem */
osSemaphoreId_t PSx_SIG_BinSemHandle;
const osSemaphoreAttr_t PSx_SIG_BinSem_attributes = {
  .name = "PSx_SIG_BinSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void debugcansend(int8_t * tmp)
{
	int8_t buf[8]={0,};
	memcpy(buf, tmp, sizeof(tmp));

	sendCan(0x3e8, buf, 8, 0);//(uint32_t ID, uint8_t data[8], uint8_t len, uint8_t ext)
}
void Cal_Real_cmd(void)
{

	double tempL;
	double tempR;

	tempL=(double)(Tmp_cmd_FL+Tmp_cmd_RL)/(2*10);
	tempR=-(double)(Tmp_cmd_FR+Tmp_cmd_RR)/(2*10);

	if(angle_rad_c == 0){

	//Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
	Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
	//printf("%d:Real_cmd_v_x 11 %f %f %f %f\n", osKernelGetTickCount(), Real_cmd_v_x, tempL, tempL, cos(ANGLE_RAD_A));

	}
	else{
		if((tempL<tempR)  &&  ((tempL>0) && (tempR>0))){
			if((sin(real_angle_c)<0.1) && (sin(real_angle_c)>-0.1))
			{
				Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
				//printf("%d:Real_cmd_v_x 221 %f %f %f %f %f\n", osKernelGetTickCount(), Real_cmd_v_x, tempL, tempL, sin(real_angle_i), sin(real_angle_c));
			}
			else{
				Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(real_angle_i)/sin(real_angle_c))*tempL)
								+((sin(real_angle_o)/sin(real_angle_c))*tempR));
				//printf("%d:Real_cmd_v_x 222 %f %f %f %f %f\n", osKernelGetTickCount(), Real_cmd_v_x, tempL, tempL, sin(real_angle_i), sin(real_angle_c));
			}

		}

		else if((tempL>tempR)  &&  ((tempL>0) && (tempR>0))){
			if((sin(real_angle_c)<0.1) && (sin(real_angle_c)>-0.1)){
				Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
							//	printf("%d:Real_cmd_v_x 331 %f %f %f %f %f\n", osKernelGetTickCount(), Real_cmd_v_x, tempL, tempL, sin(real_angle_i), sin(real_angle_c));
			}
			else{
			Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(real_angle_i)/sin(real_angle_c))*tempR)
							+((sin(real_angle_o)/sin(real_angle_c))*tempL));
			//printf("%d:Real_cmd_v_x 332 %f %f %f %f %f\n", osKernelGetTickCount(), Real_cmd_v_x, tempL, tempL, sin(real_angle_i), sin(real_angle_c));
			}
		}

		else if((tempL<tempR)  &&  ((tempL<0) && (tempR<0))){
			if((sin(real_angle_c)<0.1) && (sin(real_angle_c)>-0.1)){
				Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
				//				printf("%d:Real_cmd_v_x 441 %f %f %f %f %f\n", osKernelGetTickCount(), Real_cmd_v_x, tempL, tempL, sin(real_angle_i), sin(real_angle_c));
			}
			else{
			Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(real_angle_i)/sin(real_angle_c))*tempR)
				+((sin(real_angle_o)/sin(real_angle_c))*tempL));
			//printf("%d:Real_cmd_v_x 442 %f\n", osKernelGetTickCount(), Real_cmd_v_x);
			}
		}

		else if((tempL>tempR)  &&  ((tempL<0) && (tempR<0))){
			if((sin(real_angle_c)<0.1) && (sin(real_angle_c)>-0.1)){
				Real_cmd_v_x = C_2PIRxINv60*((tempL+tempR)/2)*fabs(cos(ANGLE_RAD_A));
			//	printf("%d:Real_cmd_v_x 551 %f %f %f %f %f\n", osKernelGetTickCount(), Real_cmd_v_x, tempL, tempL, sin(real_angle_i), sin(real_angle_c));
			}
			else{
			Real_cmd_v_x = (C_2PIRxINv60/2)*(((sin(real_angle_i)/sin(real_angle_c))*tempL)
				+((sin(real_angle_o)/sin(real_angle_c))*tempR));
		//	printf("%d:Real_cmd_v_x 552 %f\n", osKernelGetTickCount(), Real_cmd_v_x);
			}
		}
	}

	if((Tmp_cmd_FL>=0) && (Tmp_cmd_FR>=0)  ||  (Tmp_cmd_FL<=0) && (Tmp_cmd_FR<=0))//mode C
	{
		Real_cmd_w = -(CONSTANT_C_AxC_V*((tempL-tempR)/2));
	}
	else//mode B
	{
//		Real_cmd_w = (C_4PIRxINv60WB*((tempL+tempR)/2)*fabs(sin(angle_rad_c)))*1000;

		if		((tempL<tempR)  &&  ((tempL>0) && (tempR>0))){Real_cmd_w = ((Real_cmd_v_x*sin(real_angle_c))/230)*1000;}
		else if	((tempL>tempR)  &&  ((tempL>0) && (tempR>0))){Real_cmd_w = -((Real_cmd_v_x*sin(real_angle_c))/230)*1000;}
		else if	((tempL<tempR)  &&  ((tempL<0) && (tempR<0))){Real_cmd_w = -((Real_cmd_v_x*sin(real_angle_c))/230)*1000;}
		else if	((tempL>tempR)  &&  ((tempL<0) && (tempR<0))){Real_cmd_w = ((Real_cmd_v_x*sin(real_angle_c))/230)*1000;}
		printf("%d:Real_cmd_ww 552 %d %d %d %d %d\n", osKernelGetTickCount(),
				(int)Real_cmd_w, (int )Real_cmd_v_x, (int)(real_angle_c*1000), (int)(real_angle_i*1000), (int)(real_angle_o*1000));
	}

	sendcanbuf[5] = (((int16_t)(Real_cmd_w)))>>8 & 0xff;
	sendcanbuf[4] = (int16_t)(Real_cmd_w)&0xff;
	sendcanbuf[3] = (((int16_t)(Real_cmd_v_y)))>>8 & 0xff;
	sendcanbuf[2] = (int16_t)(Real_cmd_v_y)&0xff;
	sendcanbuf[1] = (((int16_t)(Real_cmd_v_x)))>>8 & 0xff;
	sendcanbuf[0] = (int16_t)(Real_cmd_v_x)&0xff;
}


int32_t Stopflagcheck(uint8_t RW, uint8_t value)
{
	printf("%d:Stopflagcheck\n", osKernelGetTickCount());
	if(osMutexWait(Stop_flagHandle, osWaitForever)==osOK)
	{
		if(RW){
			if(value == 0){Stop_flag = 0;}
			else {Stop_flag++;}
			if(Stop_flag>0xfffffff0){Stop_flag = 1;}
			osMutexRelease(Stop_flagHandle);
		}
		else {
			osMutexRelease(Stop_flagHandle);
			return Stop_flag;
		}
	}
}

int16_t Deg2Ste(uint8_t RW, int16_t deg, uint8_t num)
{
	printf("%d:Deg2Ste\n", osKernelGetTickCount());
	if(num>4){
		//printf("%d:osError\n", osKernelGetTickCount());
		return 0;}
	if(osMutexWait(DegmsgHandle, osWaitForever)==osOK)
	{
		if(RW){//write
			SteDeg[num] = deg; //printf("%d:deg in mut:%d \n", osKernelGetTickCount(), SteDeg);
			osMutexRelease(DegmsgHandle);
			return 1;
		}
		else{//read
			osMutexRelease(DegmsgHandle);
			return SteDeg[num];
		}
	}
	else{
		//printf("%d:osError\n", osKernelGetTickCount());
		return 0;
	}
}

int16_t rad2deg(double radian)
{
    return (int16_t)(radian*180/MATH_PI);
}

double deg2rad(int16_t degree)
{
    return (double)(degree*MATH_PI/180);
}

void ModeSelect(void){
	//ModeA//for linearX x linearY
	//ModeB//for linearX x angle
	//ModeC//for rotate
	//ModeD//for stop
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


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
void StartTask07(void *argument);
void VelStopTimerCallback(void *argument);
void EndModeDTimerCallback(void *argument);
void SendCanTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of canmsg */
  canmsgHandle = osMutexNew(&canmsg_attributes);

  /* creation of Degmsg */
  DegmsgHandle = osMutexNew(&Degmsg_attributes);

  /* creation of Stop_flag */
  Stop_flagHandle = osMutexNew(&Stop_flag_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of PSx_SIG_BinSem */
  PSx_SIG_BinSemHandle = osSemaphoreNew(1, 1, &PSx_SIG_BinSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of VelStopTimer */
  VelStopTimerHandle = osTimerNew(VelStopTimerCallback, osTimerPeriodic, NULL, &VelStopTimer_attributes);

  /* creation of EndModeDTimer */
  EndModeDTimerHandle = osTimerNew(EndModeDTimerCallback, osTimerOnce, NULL, &EndModeDTimer_attributes);

  /* creation of SendCanTimer */
  SendCanTimerHandle = osTimerNew(SendCanTimerCallback, osTimerPeriodic, NULL, &SendCanTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(VelStopTimerHandle, 1000);
  osTimerStart(SendCanTimerHandle, 100);
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

  /* creation of steeringtask */
  steeringtaskHandle = osThreadNew(StartTask07, NULL, &steeringtask_attributes);

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


	//printf("uxHighWaterMark: %d\n", uxTaskGetStackHighWaterMark( NULL ));

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
//	int16_t Tar_cmd_v_x = 0;
//	int16_t Tar_cmd_v_y = 0;
//	int16_t Tar_cmd_w = 0;



	uint8_t torqueSW = 0;
	uint8_t Oncetimer = 1;
	uint8_t tempflag = 0;

	//////////////////////////////
	uint32_t lastTime;



	osDelay(3000);//must delay for nmt from motor driver
	while(!(STinitdone)){osDelay(100);;}

	CanInit(FILTERID,MASKID,STDID);//must be to use it
	CAN_enableirq();


	PDOMapping(1, RxPDO0, vel_RxPDO0, 1);
	PDOMapping(2, RxPDO0, vel_RxPDO0, 1);

	PDOMapping(1, TxPDO0, vel_TxPDO0, 1);//event time mode 100ms
	PDOMapping(2, TxPDO0, vel_TxPDO0, 1);//event time mode
	PDOMapping(1, TxPDO1, vel_TxPDO1, 1);//inhibit mode 100ms
	PDOMapping(2, TxPDO1, vel_TxPDO1, 1);//inhibit mode

	for(int i=0;i<2;i++){
		SDOMsg(i+1,0x2010, 0x0, 0x01, 1);//Node_id, index,  subindex,  msg,  len//save eeprom
		SDOMsg(i+1,0x6060, 0x0, 0x03, 1);//Node_id, index,  subindex,  msg,  len//3: Profile velocity mode;
		Tor_OnOff(TORQUEON);
		SDOMsg(i+1,0x200f, 0x0, 0x01, 2);//Node_id, index,  subindex,  msg,  len//1e: Synchronization control
	}

  /* Infinite loop */
	//printf("%d: format\n", osKernelGetTickCount());
	lastTime = osKernelGetTickCount ();
  for(;;)
  {
	printf("%d: t02 001\n", osKernelGetTickCount());
	lastTime += PERIOD_CANCOMM;;
	printf("%d: t02 002\n", osKernelGetTickCount());
	osDelayUntil(lastTime);
	printf("%d: t02 003\n", osKernelGetTickCount());
	//osDelay(10);
	//printf("%d: t02\n", osKernelGetTickCount());

	if(FLAG_RxCplt>0)	//real time, check stdid, extid
	{
		printf("%d: t02 004\n", osKernelGetTickCount());
		while(FLAG_RxCplt>0){
			printf("%d: t02 005\n", osKernelGetTickCount());
			FLAG_RxCplt--;
			printf("%d: t02 006\n", osKernelGetTickCount());
			for(int i=0;i<8;i++){canbuf[i] = g_uCAN_Rx_Data[FLAG_RxCplt][i];}
			printf("%d: t02 007\n", osKernelGetTickCount());
		//	printf("canbuf: %d %d %d %d %d %d %d %d\n", canbuf[0], canbuf[1], canbuf[2], canbuf[3], canbuf[4], canbuf[5], canbuf[6], canbuf[7]);
			//printf("%dcanid: %d %d %d\n", osKernelGetTickCount(), g_tCan_Rx_Header[FLAG_RxCplt].StdId, g_tCan_Rx_Header[FLAG_RxCplt].ExtId, g_tCan_Rx_Header[FLAG_RxCplt].Timestamp);
			if(g_tCan_Rx_Header[FLAG_RxCplt].StdId>g_tCan_Rx_Header[FLAG_RxCplt].ExtId){CanId = g_tCan_Rx_Header[FLAG_RxCplt].StdId;}//�????????????????체크
			else {CanId = g_tCan_Rx_Header[FLAG_RxCplt].ExtId;}
			printf("%d: t02 008\n", osKernelGetTickCount());
			switch(CanId)//parse
			{
				case 0x3E9:
					temp_x = (((int16_t)canbuf[1])<<8) | ((int16_t)canbuf[0])&0xff;
					temp_y = (((int16_t)canbuf[3])<<8) | ((int16_t)canbuf[2])&0xff;
					temp_w = (((int16_t)canbuf[5])<<8) | ((int16_t)canbuf[4])&0xff;
					state_stop = canbuf[7];
					Tar_cmd_v_x = (double)temp_x;
					Tar_cmd_v_y = (double)temp_y;
					Tar_cmd_w = (double)temp_w;
					torqueSW = canbuf[6];
					printf("%d: t02 009 %d %d %d\n", osKernelGetTickCount(),temp_x,temp_y,temp_w);
					//if(Stop_flag++>255){Stop_flag = 1;}
					Stopflagcheck(Xbot_W, 1);
					printf("%d: t02 010\n", osKernelGetTickCount());
					//printf("%d: 0x3E9:%d %d\n", osKernelGetTickCount(),Stop_flag,Pre_Stop_flag);
					//printf("%d: Stop_flag: %d\n", osKernelGetTickCount(), Stop_flag);
					break;

				case 0x181:
					Tmp_cmd_FL = (((int16_t)canbuf[1])<<8) | ((int16_t)canbuf[0])&0xff;
					Tmp_cmd_FR = (((int16_t)canbuf[3])<<8) | ((int16_t)canbuf[2])&0xff;
					//printf("0x181 %d\n", Tmp_cmd_FL);
					break;

				case 0x182:
					Tmp_cmd_RL = (((int16_t)canbuf[1])<<8) | ((int16_t)canbuf[0])&0xff;
					Tmp_cmd_RR = (((int16_t)canbuf[3])<<8) | ((int16_t)canbuf[2])&0xff;
					break;

				case 2002:

					break;
			}
			printf("%d: t02 011\n", osKernelGetTickCount());
			g_tCan_Rx_Header[FLAG_RxCplt].StdId=0;
			g_tCan_Rx_Header[FLAG_RxCplt].ExtId=0;
			CanId = 0;

			for(int i=0;i<8;i++){canbuf[i]=0;}
		}

	}

	if((temp_w && (temp_x==0) && (temp_y==0)) || ( fabs((Tar_cmd_v_x*1000)/Tar_cmd_w)<LIMIT_MODE_C) ){//MODE C
	//if(Tar_cmd_w){

		printf("%d: t02 012 %d %d %d\n", osKernelGetTickCount(),(int)(Tar_cmd_v_x*1000),(int)(Tar_cmd_w*1000),(int)((Tar_cmd_v_x*1000)/Tar_cmd_w));
		if((Pre_ModeABCD!=ModeABCD) || (EndMode==0)){
			//printf("111osTimerStart: %d, %d\n", ModeABCD, Pre_ModeABCD);
			Pre_ModeABCD = ModeABCD;
			Tar_cmd_RR = Tar_cmd_RL = Tar_cmd_FR = Tar_cmd_FL=0;
			if(timerflag){
				//printf("timerflag: %d\n", timerflag);
				printf("%d: t02 013\n", osKernelGetTickCount());
				osTimerStart(EndModeDTimerHandle, ENDMODETIME);
				printf("%d: t02 014\n", osKernelGetTickCount());
				timerflag = 0;
				EndMode = 0;
			}
		}
		else {
			ModeABCD = 3;
			Tar_cmd_v_x=0;
			Tar_cmd_v_y=0;
			printf("%d: t02 015\n", osKernelGetTickCount());
			Tar_cmd_FL = -1*((Tar_cmd_w*CONSTANT_C_AxC_V)/SIGNIFICANT_FIGURES);

			if(Tar_cmd_FL>LIMIT_W){Tar_cmd_FL=LIMIT_W;}
			if(Tar_cmd_FL<-LIMIT_W){Tar_cmd_FL=-LIMIT_W;}
			Tar_cmd_RR = Tar_cmd_RL = Tar_cmd_FR = Tar_cmd_FL;
			printf("%d: t02 016\n", osKernelGetTickCount());
			for(int i=0;i<4;i++){Deg2Ste(Xbot_W,rad2deg(ANGLE_VEL), i);}
			printf("%d: t02 017\n", osKernelGetTickCount());
			Cal_Real_cmd();
			printf("%d: t02 018\n", osKernelGetTickCount());
		}
	}

	//else if(Tar_cmd_v_x|Tar_cmd_v_y){
//	else if((Tar_cmd_v_x|Tar_cmd_v_y) && (Tar_cmd_w==0)){
//
//		ModeABCD = 1;
//
//		if((Pre_ModeABCD!=ModeABCD) || (EndMode==0)){
//			//printf("111osTimerStart: %d, %d\n", ModeABCD, Pre_ModeABCD);
//			Pre_ModeABCD = ModeABCD;
//			Tar_cmd_RR = Tar_cmd_RL = Tar_cmd_FR = Tar_cmd_FL=0;
//			if(timerflag){
//				//printf("timerflag: %d\n", timerflag);
//				osTimerStart(EndModeDTimerHandle, ENDMODETIME);
//				timerflag = 0;
//				EndMode = 0;
//			}
//		}
//		else{
//			Tar_cmd_FL = CONSTANT_VEL  *  (Tar_cmd_v_x*cos(ANGLE_RAD_A) + Tar_cmd_v_y*sin(ANGLE_RAD_A));
//
//			if(Tar_cmd_FL>LIMIT_V){Tar_cmd_FL=LIMIT_V;}
//			if(Tar_cmd_FL<-LIMIT_V){Tar_cmd_FL=-LIMIT_V;}
//			Tar_cmd_FR = -Tar_cmd_FL;
//			Tar_cmd_RL = Tar_cmd_FL;
//			Tar_cmd_RR = -Tar_cmd_FL;
//
//			if(Tar_cmd_v_x<0){
//				Tar_cmd_FL*=-1;
//				Tar_cmd_FR*=-1;
//				Tar_cmd_RL*=-1;
//				Tar_cmd_RR*=-1;
//			}
//		}
//		//SteDeg=rad2deg(ANGLE_RAD);
//		Deg2Ste(Xbot_W,rad2deg(ANGLE_RAD_A));
//
//		Cal_Real_cmd();
//	}


	//else if(Tar_cmd_v_x && (Tar_cmd_w!=0)){
		else {	//mode2
			printf("%d: t02 019\n", osKernelGetTickCount());
			if((Pre_ModeABCD!=ModeABCD) || (EndMode==0)){
				printf("%d: t02 020\n", osKernelGetTickCount());
				//printf("%d:111osTimerStart: %d, %d, %d\n", osKernelGetTickCount(), ModeABCD, Pre_ModeABCD, EndMode);
				Pre_ModeABCD = ModeABCD;
				Tar_cmd_RR = Tar_cmd_RL = Tar_cmd_FR = Tar_cmd_FL=0;
				angle_rad_i = 0;
				angle_rad_o = 0;
				printf("%d:send angle2 %d %d %d %d %d %d %d %d %d %d %d %d %d\n", osKernelGetTickCount(),
									SteDeg[0]*100, SteDeg[1]*100, SteDeg[2]*100, SteDeg[3]*100, (int)(angle_rad_i*1000), (int)(angle_rad_o*1000),
									SAngle[0],SAngle[1],SAngle[2],SAngle[3], temp_x, temp_y, temp_w);
				if(timerflag){
					printf("%d: t02 021\n", osKernelGetTickCount());
					//printf("timerflag: %d\n", timerflag);
					osTimerStart(EndModeDTimerHandle, ENDMODETIME);
					printf("%d: t02 022\n", osKernelGetTickCount());
					timerflag = 0;
					EndMode = 0;
					printf("%d:send angle3 %d %d %d %d %d %d %d %d %d %d %d %d %d\n", osKernelGetTickCount(),
										SteDeg[0]*100, SteDeg[1]*100, SteDeg[2]*100, SteDeg[3]*100, (int)(angle_rad_i*1000), (int)(angle_rad_o*1000),
										SAngle[0],SAngle[1],SAngle[2],SAngle[3], temp_x, temp_y, temp_w);
				}
			}
			else{	//mode2
			//Tar_cmd_FL = CONSTANT_VEL  *  (Tar_cmd_v_x*cos(ANGLE_RAD_B) + Tar_cmd_v_y*sin(ANGLE_RAD_B));
			//printf("%d:222osTimerStart: %d, %d, %d\n", osKernelGetTickCount(), ModeABCD, Pre_ModeABCD, EndMode);
			if(Tar_cmd_v_x>LIMIT_V){Tar_cmd_v_x=LIMIT_V;}
			if(Tar_cmd_v_x<-LIMIT_V){Tar_cmd_v_x=-LIMIT_V;}
			printf("%d: t02 023\n", osKernelGetTickCount());
			//if((sin(real_angle_c)<0.1) && (sin(real_angle_c)>-0.1)){
//			if((Tar_cmd_v_x<0.1) && (Tar_cmd_v_x>-0.1)){
//				angle_rad_c = 1;
//
//			}
//			else { angle_rad_c = fabs(asin(((230*Tar_cmd_w) / (Tar_cmd_v_x*1000)))); }
			printf("%d: t02 024\n", osKernelGetTickCount());
			angle_rad_c = fabs(asin(((230*Tar_cmd_w) / (Tar_cmd_v_x*1000))));

			angle_rad_i = fabs(atan2(230,(230 / tan(angle_rad_c))-209.5));
			angle_rad_o = fabs(atan2(230,(230 / tan(angle_rad_c))+209.5));

			Tar_cmd_v_i = (Tar_cmd_v_x*sin(angle_rad_c)) / sin(angle_rad_i);
			Tar_cmd_v_o = (Tar_cmd_v_x*sin(angle_rad_c)) / sin(angle_rad_o);

			printf("%d: t02 025\n", osKernelGetTickCount());
			printf("%d:send angle1 %d %d %d %d %d %d %d %d %d %d %d %d %d\n", osKernelGetTickCount(),
								SteDeg[0]*100, SteDeg[1]*100, SteDeg[2]*100, SteDeg[3]*100, (int)(angle_rad_i*1000), (int)(angle_rad_o*1000),
								SAngle[0],SAngle[1],SAngle[2],SAngle[3], temp_x, temp_y, temp_w);
			if(temp_w==0){
				printf("%d: t02 026\n", osKernelGetTickCount());
				Tar_cmd_v_i=Tar_cmd_v_o=Tar_cmd_v_x;
				angle_rad_i=angle_rad_o=angle_rad_c=0;

				Tar_cmd_FL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_RL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				printf("%d: t02 027\n", osKernelGetTickCount());
				Deg2Ste(Xbot_W,0, STMotorID1);
				Deg2Ste(Xbot_W,0, STMotorID2);
				Deg2Ste(Xbot_W,0, STMotorID3);
				Deg2Ste(Xbot_W,0, STMotorID4);
				printf("%d: t02 028\n", osKernelGetTickCount());
			}

			if((temp_w>0) && (temp_x>0)){
				printf("%d: t02 029\n", osKernelGetTickCount());
				Tar_cmd_FL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_RL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_o);

				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID1);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID2);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID3);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID4);
				printf("%d: t02 030\n", osKernelGetTickCount());

			}

			else if((temp_w<0) && (temp_x>0)){
				printf("%d: t02 031\n", osKernelGetTickCount());
				Tar_cmd_FL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				Tar_cmd_RL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				printf("%d: t02 032\n", osKernelGetTickCount());
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID1);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID2);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID3);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID4);
				printf("%d: t02 033\n", osKernelGetTickCount());
			}

			else if((temp_w>0) && (temp_x<0)){
				printf("%d: t02 034\n", osKernelGetTickCount());
				Tar_cmd_FL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				Tar_cmd_RL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				printf("%d: t02 035\n", osKernelGetTickCount());
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID1);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID2);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID3);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID4);
				printf("%d: t02 036\n", osKernelGetTickCount());
			}

			else if((temp_w<0) && (temp_x<0)){
				printf("%d: t02 037\n", osKernelGetTickCount());
				Tar_cmd_FL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_RL = (int16_t)(C_60xINv2PIR * Tar_cmd_v_i);
				Tar_cmd_FR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_o);
				Tar_cmd_RR = -(int16_t)(C_60xINv2PIR * Tar_cmd_v_o);

				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID1);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID2);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_o), STMotorID3);
				Deg2Ste(Xbot_W,rad2deg(angle_rad_i), STMotorID4);
				printf("%d: t02 038\n", osKernelGetTickCount());
			}

			ModeABCD = 2;//B mode
		}


		Cal_Real_cmd();
		printf("%d: t02 039\n", osKernelGetTickCount());
	}

	if(((temp_x==0) && (temp_y==0) && (temp_w==0))  ||  (Stopflagcheck(Xbot_R, 1)==0))
	{
		printf("%d: t02 040\n", osKernelGetTickCount());
		ModeABCD = 4;//temp
		Pre_ModeABCD = 4;//temp
		Tar_cmd_RR = Tar_cmd_RL = Tar_cmd_FR = Tar_cmd_FL=0;

		//for(int i=0;i<4;i++){Deg2Ste(Xbot_W,rad2deg(ANGLE_VEL), i);}
		printf("%d: t02 041\n", osKernelGetTickCount());
		Cal_Real_cmd();
		printf("%d: t02 042\n", osKernelGetTickCount());
	}


	sendcanbuf[7] = VERSION_MINOR;
	sendcanbuf[6] = VERSION_MAJOR;
	printf("%d:send angle  %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", osKernelGetTickCount(),
						SteDeg[0]*100, SteDeg[1]*100, SteDeg[2]*100, SteDeg[3]*100, (int)(angle_rad_i*1000), (int)(angle_rad_o*1000),
						SAngle[0],SAngle[1],SAngle[2],SAngle[3], temp_x, temp_y, temp_w, state_stop);
  }
  printf("%d: t02 043\n", osKernelGetTickCount());
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
	uint32_t lastTime;
	uint8_t Dir_Rot = 0; //direction of rotation
	uint8_t FT_flag = 0; //FineTuning_flag
	uint8_t send_flag = 0; //FineTuning_flag
	uint8_t set_flag = 0; //FineTuning_flag

	int32_t angle = 0;
	int32_t pre_angle = 0;
	int32_t speed_angle = 0;

	int16_t pre_SteDeg[4] = {0,};	//steering degree unit=0.01 degree
	int16_t start_SteDeg[4] = {0,};
	int16_t end_SteDeg[4] = {0,};
//	int32_t SAngle[4] = {0,};
//	char buf[48]={	 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12,		//1 front right
//					13, 14, 15, 16, 17, 18, 19, 20, 21, 22,	23, 24,		//2 front left
//					25, 26, 27, 28, 29, 30, 31, 32,	33, 34, 35, 36,		//3 rear right
//					37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48	};	//4 rear left

	osDelay(1000);
	GPIO_enableirq();
	osDelay(100);
	osThreadFlagsSet(IRQ_PSxHandle, 1);

	for(int i=0;i<4;i++){
		if(HAL_GPIO_ReadPin(GPIOA, ((1<<i)<<4))){//GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
			if((i==STMotorID2) || (i==STMotorID3)) 	{Dir_Rot = SERVO_CW;}
			else					{Dir_Rot = SERVO_CCW;FT_flag |= (1<<i);}
		}
		else {
			if((i==STMotorID2) || (i==STMotorID3))	{Dir_Rot = SERVO_CCW;FT_flag |= (1<<i);}
			else					{Dir_Rot = SERVO_CW;}
		}
		DataSetSteering(buf, i, Dir_Rot, RPM_1, SERVO_INIT, INIT_SPEED);// i= STMotorIDx, x=1~4
		printf("PS_SIG1_Pin ccw init. %d %x\n", FT_flag, ((1<<i)<<4));
	}

	osDelay(1000);

	for(int i=0;i<40;i++){
		osDelay(200);
		ServoMotor_writeDMA(buf);//servo init. must done init within 500*20ms
		printf("%d ", i);
		if(STinitdone){printf("steering origin init done!!!.\n"); break;}
		if(i==39){
			HAL_Delay(100);
			printf("steering origin init failed reset!!!!.\n");
			HAL_Delay(100);
			//NVIC_SystemReset();
		}
	}
	osDelay(500);
	STinitdone = 0;
	//EndInit = 0;
	//GPIO_enableirq();
//	osThreadFlagsSet(IRQ_PSxHandle, 1);
	printf("%d: osTFSet\n", osKernelGetTickCount());

	for(int i=0;i<4;i++){
		if(FT_flag&(1<<i)){
			DataSetSteering(buf, i, SERVO_CW, STM_FT_ID[i][SERVO_CW], SERVO_POS, INIT_SPEED);
			printf("SERVO_cW\n");
		}
		else {
			DataSetSteering(buf, i, SERVO_CCW, STM_FT_ID[i][SERVO_CCW], SERVO_POS, INIT_SPEED);
			printf("SERVO_ccW\n");
		}
		PS_SIGx_Pin |= (1<<i);
	}

	for(int i=0;i<10;i++){
		ServoMotor_writeDMA(buf);//servo init. must done init within 500*20ms
		osDelay(500);
		}

	for(int i=0;i<4;i++){
		if(FT_flag&(1<<i)){
			DataSetSteering(buf, i, SERVO_CW, 0, SERVO_INIT, INIT_SPEED);
			printf("SERVO_cW\n");
		}
		else {
			DataSetSteering(buf, i, SERVO_CCW, 0, SERVO_INIT, INIT_SPEED);
			printf("SERVO_ccW\n");
		}
		PS_SIGx_Pin |= (1<<i);
	}

	for(int i=0;i<10;i++){
		ServoMotor_writeDMA(buf);//servo init. must done init within 500*20ms
		osDelay(500);
		}


	Dir_Rot = 0;//init

	HAL_UART_Receive_IT(&huart3, tmp_rx[0] , 12);
	lastTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	lastTime += PERIOD_STEERING;
	osDelayUntil(lastTime);

	printf("%d: t03\n", osKernelGetTickCount());

	if(ModeABCD == 1){
#if 0
		if(Deg2Ste(Xbot_R,0) == 180||Deg2Ste(Xbot_R,0) == -180){Deg2Ste(Xbot_W,0);}//forward, rear
		if(Tar_cmd_v_x==0&&Tar_cmd_v_y>0){Deg2Ste(Xbot_W,90); Dir_Rot=SERVO_CCW;}//left
		else if(Tar_cmd_v_x==0&&Tar_cmd_v_y<0){Deg2Ste(Xbot_W,90); Dir_Rot=SERVO_CW;}//right

		if		((Tar_cmd_v_x>0) && (Tar_cmd_v_y>0)){/*SteDeg*=1;*/							Dir_Rot=SERVO_CCW; }//the first quadrant
		else if	((Tar_cmd_v_x<0) && (Tar_cmd_v_y>0)){Deg2Ste(Xbot_W,180-Deg2Ste(Xbot_R,0)); Dir_Rot=SERVO_CCW; }//the second quadrant
		else if	((Tar_cmd_v_x<0) && (Tar_cmd_v_y<0)){Deg2Ste(Xbot_W,180+Deg2Ste(Xbot_R,0)); Dir_Rot=SERVO_CW; }//the third quadrant
		else if	((Tar_cmd_v_x>0) && (Tar_cmd_v_y<0)){Deg2Ste(Xbot_W,abs(Deg2Ste(Xbot_R,0))); Dir_Rot=SERVO_CW; }//the fourth quadrant

		if((SteDeg>=0) && (SteDeg<=90)){//prevent from angle over range
			if(Dir_Rot==SERVO_CW){for(int i=0;i<4;i++){angle = -1*SteDeg[i];}}
			else{for(int i=0;i<4;i++){angle = SteDeg[i];}}
			printf("%d: abs %d %d %d\n", osKernelGetTickCount(), speed_angle, pre_angle, angle);
			if(pre_angle != angle){
				speed_angle=abs(angle-pre_angle);
				printf("%d: pre_angle != angle %d %d %d\n", osKernelGetTickCount(), speed_angle, pre_angle, angle);
				pre_angle = angle;
			}

			//DataSetSteering(buf, STMotorID1, Dir_Rot, SteDeg*100, SERVO_POS,(speed_angle/9));
			DataSetSteering(buf, STMotorID1, Dir_Rot, SteDeg[0]*100, SERVO_POS,20);
			//DataSetSteering(buf, STMotorID1, Dir_Rot, SteDeg*100, -1,50);
			//DataSetSteering(buf, STMotorID1, Dir_Rot, SteDeg*100, 2, 150);
			//if(Dir_Rot==SERVO_CW)	{pre_angle = -1*pre_angle;}
			//else					{pre_angle = pre_angle;}

			DataSetSteering(buf, STMotorID2, Dir_Rot, SteDeg[1]*100, SERVO_POS,20);
			DataSetSteering(buf, STMotorID3, Dir_Rot, SteDeg[2]*100, SERVO_POS,20);
			DataSetSteering(buf, STMotorID4, Dir_Rot, SteDeg[3]*100, SERVO_POS,20);
		}
#endif
		printf("Mode A\n");
	}

	if(ModeABCD == 2){
		if(Deg2Ste(Xbot_R,0, STMotorID1) == 0){//forward, rear
			for(int i=0;i<4;i++){Deg2Ste(Xbot_W, 0, i);}
			printf("%d: abs %d\n", osKernelGetTickCount(), SteDeg[0]);
		}
//		if(Tar_cmd_v_x==0&&Tar_cmd_v_y>0){Deg2Ste(Xbot_W,90); Dir_Rot=SERVO_CCW;}//left
//		else if(Tar_cmd_v_x==0&&Tar_cmd_v_y<0){Deg2Ste(Xbot_W,90); Dir_Rot=SERVO_CW;}//right

		if		((Tar_cmd_v_x>0) && (Tar_cmd_w>0)){/*SteDeg*=1;*/							Dir_Rot=SERVO_CCW; }//the first quadrant
		else if	((Tar_cmd_v_x<0) && (Tar_cmd_w<0)){/*Deg2Ste(Xbot_W,abs(Deg2Ste(Xbot_R,0)));*/	Dir_Rot=SERVO_CCW; }//the second quadrant
		else if	((Tar_cmd_v_x<0) && (Tar_cmd_w>0)){/*Deg2Ste(Xbot_W,abs(Deg2Ste(Xbot_R,0)));*/	Dir_Rot=SERVO_CW; }//the third quadrant
		else if	((Tar_cmd_v_x>0) && (Tar_cmd_w<0)){/*Deg2Ste(Xbot_W,abs(Deg2Ste(Xbot_R,0)));*/	Dir_Rot=SERVO_CW; }//the fourth quadrant

		for(int i=0;i<4;i++){
			if(SteDeg[i]>90){Deg2Ste(Xbot_W, 90, i);}//prevent over angle
		}
//		SteDeg=rad2deg(ANGLE_VEL);
//		Deg2Ste(Xbot_W,rad2deg(ANGLE_VEL));
//		printf("%d: abs %d\n", osKernelGetTickCount(), SteDeg);


		if(pre_SteDeg[0] == SteDeg[0]){
			set_flag = 1;
			for(int i=0;i<4;i++){
				end_SteDeg[i] = ((SteDeg[i]*MS_PER_DEG)+5) / RES_SM;//+5 is round
				if(start_SteDeg[i]>end_SteDeg[i]) {SAngle[i] = start_SteDeg[i] - end_SteDeg[i];}
				else if (start_SteDeg[i]<end_SteDeg[i]) {SAngle[i] = end_SteDeg[i] - start_SteDeg[i];}
				start_SteDeg[i] = SAngle[i];
				printf("%d: input data %d, %d, %d, %d\n", osKernelGetTickCount(),
						SteDeg[i], SAngle[i], end_SteDeg[i] , start_SteDeg[i] );
			}
		}

		else{
			for(int i=0;i<4;i++){
				pre_SteDeg[i] = SteDeg[i];
				send_flag = 1;
				printf("%d: change data %d, %d, %d, %d\n", osKernelGetTickCount(),
						SteDeg[i], SAngle[i], end_SteDeg[i] , start_SteDeg[i] );
			}
		}

//		DataSetSteering(buf, STMotorID1, Dir_Rot, SteDeg[0]*100, SERVO_POS, 20);
//		DataSetSteering(buf, STMotorID2, Dir_Rot, SteDeg[1]*100, SERVO_POS, 20);
//		DataSetSteering(buf, STMotorID3, Dir_Rot^1, SteDeg[2]*100, SERVO_POS, 20);
//		DataSetSteering(buf, STMotorID4, Dir_Rot^1, SteDeg[3]*100, SERVO_POS, 20);
		DataSetSteering(buf, STMotorID1, Dir_Rot, SteDeg[0]*100, SERVO_POS, SAngle[0]);
		DataSetSteering(buf, STMotorID2, Dir_Rot, SteDeg[1]*100, SERVO_POS, SAngle[1]);
		DataSetSteering(buf, STMotorID3, Dir_Rot^1, SteDeg[2]*100, SERVO_POS, SAngle[2]);
		DataSetSteering(buf, STMotorID4, Dir_Rot^1, SteDeg[3]*100, SERVO_POS, SAngle[3]);
		printf("%d: MM %d\n", osKernelGetTickCount(), SteDeg[0]);
	}

	if(ModeABCD == 3){
//		SteDeg=rad2deg(ANGLE_VEL);
//		for(int i=0;i<4;i++){Deg2Ste(Xbot_W,rad2deg(ANGLE_VEL), i);}
		send_flag = 1;
		set_flag = 1;
		pre_SteDeg[0] = 1; //for set send_flag of mode B
		//printf("%d: abs %d\n", osKernelGetTickCount(), SteDeg);
		DataSetSteering(buf, STMotorID1, SERVO_CCW, SteDeg[0]*100, SERVO_POS, 20);
		DataSetSteering(buf, STMotorID2, SERVO_CW, SteDeg[1]*100, SERVO_POS, 20);
		DataSetSteering(buf, STMotorID3, SERVO_CW, SteDeg[2]*100, SERVO_POS, 20);
		DataSetSteering(buf, STMotorID4, SERVO_CCW, SteDeg[3]*100, SERVO_POS, 20);
		for(int i=0;i<4;i++){
			SAngle[i] = ((SteDeg[i]*MS_PER_DEG)+5) / RES_SM;
		}
		printf("Mode c\n");
	}

	if(ModeABCD == 4){
		send_flag = 1;
		set_flag = 1;
		pre_SteDeg[0] = 1; //for set send_flag of mode B
//		SteDeg=rad2deg(ANGLE_VEL);
//		for(int i=0;i<4;i++){Deg2Ste(Xbot_W,rad2deg(ANGLE_VEL), i);}
		//DataSetSteering(buf, STMotorID1, SERVO_CW, SteDeg*100, SERVO_POS, 20);
		//DataSetSteering(buf, STMotorID1, SERVO_CW, SteDeg*100, 2, 250); pre_angle = -1*SteDeg;
		DataSetSteering(buf, STMotorID1, SERVO_CW, SteDeg[0]*100, SERVO_POS,20);
		DataSetSteering(buf, STMotorID2, SERVO_CCW, SteDeg[1]*100, SERVO_POS, 20);
		DataSetSteering(buf, STMotorID3, SERVO_CCW, SteDeg[2]*100, SERVO_POS, 20);
		DataSetSteering(buf, STMotorID4, SERVO_CW, SteDeg[3]*100, SERVO_POS, 20);
//		EndModeD = 0;
		//osDelay(10);
		for(int i=0;i<4;i++){
			SAngle[i] = ((SteDeg[i]*MS_PER_DEG)+5) / RES_SM;
		}
		printf("Mode D\n");
	}
	//osDelay(10);
#if 1//testing
	if((send_flag==1) && (set_flag==1)){
		ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		send_flag = 0;
		set_flag = 0;
		for(int i=0;i<4;i++){
			printf("[%d] %d ", i, SAngle[i]);
			start_SteDeg[i] = 0;
			end_SteDeg[i] = 0;
//			SAngle[i] = 0;
		}

		printf("%d: writeDMA %d %d %d %d %d %d %d %d\n", osKernelGetTickCount(),SteDeg[0],SteDeg[1],SteDeg[2],SteDeg[3],SAngle[0],SAngle[1],SAngle[2],SAngle[3]);
	}

#else
	//origin
	ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
#endif
	osDelay(5); DataReadSteering(STMotorID1, 0xA1);
	osDelay(5); DataReadSteering(STMotorID2, 0xA1);
	osDelay(5); DataReadSteering(STMotorID3, 0xA1);
	osDelay(5); DataReadSteering(STMotorID4, 0xA1);

	//printf("uxHighWaterMark: %d\n", uxTaskGetStackHighWaterMark( NULL ));//check #define INCLUDE_uxTaskGetStackHighWaterMark 1
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

	ws2812AllColor(70,70,70);//r, g, b
	ws2812NumOn(NUM_NPLED);

  /* Infinite loop */
  for(;;)
  {
		lastTime += PERIOD_NP_LED;
		osDelayUntil(lastTime);

		temp++;
		switch (temp) {
			case 1:
				//printf("case1\n");
				//ws2812SetColor(0,0,0,1);//index, r, g, b
				break;

			case 2:
				//printf("case2\n");
				//ws2812SetColor(7,0,0,1);//index, r, g, b
				break;

			case 3:
				//printf("case3\n");
				//ws2812SetColor(6,0,0,1);//index, r, g, b
				break;
		}


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
	fanOn(100);
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
  /* USER CODE BEGIN StartTask06 */
//	uint8_t EndInit = 0;
	//uint32_t lastTime = osKernelGetTickCount();
	//osDelay(10);//for printf();
	printf("StartTask06 %d: \n", PS_SIGx_Pin);
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
	//  printf("%d: t06\n", osKernelGetTickCount());
	if(PS_SIGx_Pin&1){//1ch init
		PS_SIGx_Pin &= ~(1); printf(" PS_SIG1_stop.\n");
		EndInit |= 1;
		DataSetSteering(buf, 0, SERVO_CCW, 0, 0, 30);
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
	}

	if(PS_SIGx_Pin&2){//2ch init
		PS_SIGx_Pin &= ~(2); printf(" PS_SIG2_stop.\n");
		DataSetSteering(buf, 1, SERVO_CCW, 0, 0, 30);
		EndInit |= 2;
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
	}
	if(PS_SIGx_Pin&4){//3ch init
		PS_SIGx_Pin &= ~(4); printf(" PS_SIG3_stop.\n");
		DataSetSteering(buf, 2, SERVO_CCW, 0, 0, 30);
		EndInit |= 4;
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
	}	if(PS_SIGx_Pin&8){//4ch init
		PS_SIGx_Pin &= ~(8); printf(" PS_SIG4_stop.\n");
		DataSetSteering(buf, 3, SERVO_CCW, 0, 0, 30);
		EndInit |= 8;
		//ServoMotor_writeDMA(buf);//use osdelay(6)*2ea
		//for(int i=0;i<48;i++){buf[i]=0;}//clear buf
		printf("EndInit %d\n", EndInit);
	}
	if(EndInit == 15) {

		GPIO_disableirq();
		STinitdone++;
		printf("%d: EndInit == 15. %d, %d\n", osKernelGetTickCount(), osThreadFlagsWait(1, 0, osWaitForever), osThreadFlagsGet());
		//EndInit = 0;
	}
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the steeringtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
	uint8_t tmparr[4][12] = {0};
	uint8_t tempID = 0;
	uint8_t rx_checksum[4] = {0,};
	uint16_t real_angle[4] = {0,};

	osDelay(25000);//for initializing steering motor
//	HAL_UART_Receive_IT(&huart3, tmp_rx , SERVO_RXBUFLEN);

	uint32_t lastTime = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	printf("%d: t071\n", osKernelGetTickCount());
	lastTime += PERIOD_STEERING;
	osDelayUntil(lastTime);
	printf("%d: t073\n", osKernelGetTickCount());
	for(int k=0;k<4;k++){//copy data to buffer
		for(int i=0;i<12;i++){
			if(tmp_rx[k][i]==0xFF && tmp_rx[k][i+1]==0xFE)//parsing
			{
				tempID = tmp_rx[k][i+2];
				for(int j=0;j<12;j++)
				{
					if(i+j<12){tmparr[tempID][j]=tmp_rx[k][i+j];}
					else {tmparr[tempID][j]=tmp_rx[k][i+j-12];}
				}
			}
		}
	}

	printf("%d: t072\n", osKernelGetTickCount());
	if(flag_rx == 1){

		for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[0][i]);} printf("\n");
		for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[1][i]);} printf("\n");
		for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[2][i]);} printf("\n");
		for(int i=0;i<SERVO_RXBUFLEN;i++){printf("%02x ", tmparr[3][i]);} printf("\n");

		flag_rx = 0;
	}

	for(int j=0;j<4;j++){
		rx_checksum[j] = tmparr[j][2]+tmparr[j][3];//id+length

		for(int i=5;i<tmparr[j][3]+4;i++) {
			rx_checksum[j] += tmparr[j][i];
		}//checksum ~(Packet 2 + Packet 3 + Packet '5' + ?? + Packet N) [1byte]
		rx_checksum[j] ^= 0xff;//invert value. checksum done.


		if(tmparr[j][4]==rx_checksum[j]){
			real_angle[j] = tmparr[j][7]*0x100+tmparr[j][8];
			//printf("%d: angle[%d]: %03d \n", osKernelGetTickCount(), j, real_angle[j]);
		}
	}


	if(real_angle[0]>real_angle[1]){
		real_angle_i = deg2rad((double)((round((double)(real_angle[0])/100) + round((double)(real_angle[2])/100)) /2));//unit 0.01
		real_angle_o = deg2rad((double)((round((double)(real_angle[1])/100) + round((double)(real_angle[3])/100)) /2));//unit 0.01
	}

	else{
		real_angle_i = deg2rad((double)((round((double)(real_angle[1])/100) + round((double)(real_angle[3])/100)) /2));//unit 0.01
		real_angle_o = deg2rad((double)((round((double)(real_angle[0])/100) + round((double)(real_angle[2])/100)) /2));//unit 0.01
	}

	real_angle_c = (atan2(230*tan(real_angle_i),230+(209.5*tan(real_angle_i)))
				+ atan2(230*tan(real_angle_o),230-(209.5*tan(real_angle_o))))/2;

	printf("%d: angle %f %f %f %f %f\n", osKernelGetTickCount(), atan2(230*tan(real_angle_i),230+(209.5*tan(real_angle_i))),
			atan2(230*tan(real_angle_o),230-(209.5*tan(real_angle_o))), real_angle_c, real_angle_i, real_angle_o);

  }
  /* USER CODE END StartTask07 */
}

/* VelStopTimerCallback function */
void VelStopTimerCallback(void *argument)
{
  /* USER CODE BEGIN VelStopTimerCallback */

	//must be check this function
	int32_t TmpFlag = Stopflagcheck(Xbot_R, 1);

	printf("%d: VelStopTimer:%d %d\n", osKernelGetTickCount(),TmpFlag,Pre_Stop_flag);
	if(Pre_Stop_flag != TmpFlag){
		Pre_Stop_flag = TmpFlag;
		//printf("%d: VelStop1Stop_flag: %d\n", osKernelGetTickCount(), Stop_flag);
	}
	else {Stopflagcheck(Xbot_W, 0);
	printf("%d: VelStop2Stop_flag: %d\n", osKernelGetTickCount(), Stop_flag);
	}
  /* USER CODE END VelStopTimerCallback */
}

/* EndModeDTimerCallback function */
void EndModeDTimerCallback(void *argument)
{
  /* USER CODE BEGIN EndModeDTimerCallback */
	//EndModeD = 1;
	timerflag = 1;
	EndMode = 1;
  /* USER CODE END EndModeDTimerCallback */
}

/* SendCanTimerCallback function */
void SendCanTimerCallback(void *argument)
{
  /* USER CODE BEGIN SendCanTimerCallback */
	//send can message by 10hz
	printf("%d: SendCanTimerCallback1\n", osKernelGetTickCount());

	Vel_PDOMsg(1, TxPDO0, Tar_cmd_FL, Tar_cmd_FR);
	Vel_PDOMsg(2, TxPDO0, Tar_cmd_RL, Tar_cmd_RR);


	debugcansend(candbg);
	for(int i=0;i<8;i++){candbg[i]=0;}
	//printf("%d: angle_rad_c %d %d %d %f %f %d\n", osKernelGetTickCount(), rad2deg(angle_rad_c), rad2deg(angle_rad_i), rad2deg(angle_rad_o), angle_rad_i, angle_rad_o, Tar_cmd_FL);
	printf("%d: SteDeg %d %d %d %d\n", osKernelGetTickCount(), SteDeg[0], SteDeg[1], SteDeg[2], SteDeg[3]);
	sendCan(0x7D1, sendcanbuf, 8, 0);//(uint32_t ID, uint8_t data[8], uint8_t len, uint8_t ext)
	printf("%d: SteDeg %d %d %d %d %d %d %d %d\n", osKernelGetTickCount(), sendcanbuf[0], sendcanbuf[1], sendcanbuf[2],
			sendcanbuf[3], sendcanbuf[4], sendcanbuf[5], sendcanbuf[6], sendcanbuf[7]);
	for(int i=0;i<8;i++){canbuf[i]=0;}
	printf("%d: SendCanTimerCallback2\n", osKernelGetTickCount());
  /* USER CODE END SendCanTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

