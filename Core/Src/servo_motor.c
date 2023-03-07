#include "servo_motor.h"

char checksum_val = 0;
int flag_rx = 0;
int getProximity = 0;
uint8_t tmp_rx[12]={0,};
int Read_flag = 0;

extern uint8_t touch_data;
extern uint8_t uv_init_flag;

void baudrate_reinit()
{
	uint8_t tmp_tx[7]={0,};
	////9600 >>115200 motor baudrate chagne command////
	tmp_tx[0] =0xFF;
	tmp_tx[1] =0xFE;
	tmp_tx[2] =0x00;
	tmp_tx[3] =0x03;
	tmp_tx[4] =0xE8;
	tmp_tx[5] =0x07;
	tmp_tx[6] =0x0D;

	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart3, tmp_tx, 7, 100);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
	///////////////////////////////////////////////////

	///huart3 baudrate change///
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      Error_Handler();
    }
    ////////////////////////////
}
void ServoMotor_init()
{
	HAL_UART_Receive_IT(&huart3, tmp_rx, 12);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if (huart->Instance == USART3) {printf("hal_rev irq: %d\n", HAL_UART_Receive_IT(&huart3, tmp_rx, 12));
	}//SET INTERRUPT
	flag_rx = 1;
	printf("H_URCBf\n");
}

void ServoMotor_write(const uint8_t* str)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    if(Read_flag == 1){
    	HAL_NVIC_DisableIRQ(USART3_IRQn); //Rx Callback 함수 Disable
    	HAL_UART_Transmit(&huart3, str, 6, 100);
    	HAL_NVIC_EnableIRQ(USART3_IRQn);  //Rx callback 함수 enable
    }
    else {
    	HAL_NVIC_DisableIRQ(USART3_IRQn); //Rx Callback 함수 Disable
    	HAL_UART_Transmit(&huart3, str, SERVO_BUFLEN, 100);
    	HAL_NVIC_EnableIRQ(USART3_IRQn);  //Rx callback 함수 enable
    }

    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
}

void ServoMotor_writeDMA(const uint8_t* str)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    //osDelay(6);//because transmit_DMA
    if(HAL_UART_Transmit_DMA(&huart3,str, 48)!= HAL_OK){Error_Handler();}
    osDelay(6);//because transmit_DMA
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
}

void ServoMotor_control(uint8_t direction, unsigned short position, uint8_t init)
{
    //  {0xFF,0xFE,0x00,0x06,0xA2,0x02,0x00,0x23,0x28,0x0A,0,0};
    //char checksum_val = 0;
    //#define BMC_open  1,9000,0  //direction= ccw, position 90degree, normal
	uint8_t buf[12];
    buf[0]=0xFF;//header
    buf[1]=0xFE;//header
    buf[2]=0x00;//id fixed
    buf[3]=0x06;//length
    buf[4]=0x00;//checksum
    buf[5]=0x02 + init;//mode,  2=position control mode , 3=speed control mode
    buf[6]=direction;//direction ccw=0x00, cw=0x01
    buf[7]=(char)(position>>8);//position
    buf[8]=(char)position;//position
    if(init == 1){buf[9]=STOP_SPEED;}//stop speed 0.3s>>0.6s 220520>>0.8s 220621
    else buf[9]=0x1E;//speed, position second = 3s
    buf[10]=0x00;//reservation
    buf[11]=0x00;//reservation
    
    //FF FE 00 06 EC 03 00 00 00 0A
    //0  1  2  3  4  5  6  7  8  9
    for(int i=2;i<SERVO_BUFLEN;i++) {checksum_val += buf[i];}//checksum ~(Packet 2 + Packet 3 + Packet 5 + … + Packet N) [1byte]
    buf[4]=~(checksum_val);//checksum ~(Packet 2 + Packet 3 + Packet 5 + … + Packet N) [1byte]
    checksum_val=0x00;//checksum

    ServoMotor_write(buf);
    
}

void DataSetSteering(const uint8_t* str, uint8_t id, uint8_t direction, unsigned short position, uint8_t init, uint8_t speed)
{
	uint8_t buf[12];

    buf[0]=0xFF;//header
    buf[1]=0xFE;//header
    buf[2]=id;//id fixed
    buf[3]=0x06;//length
    if(init == 2){buf[3]=0x07;}
    buf[4]=0x00;//checksum
    buf[5]=0x02 + init;//mode,  2=position control mode , 3=speed control mode
    if(init == 2){buf[5]=0x01;}
    buf[6]=direction;//direction ccw=0x00, cw=0x01
    buf[7]=(char)(position>>8);//position
    buf[8]=(char)position;//position
    printf("%d: speed0 %d\n", osKernelGetTickCount(), init);
    if(init == 1){buf[9]=STOP_SPEED;buf[10]=0x00; }//stop speed 0.3s>>0.6s 220520>>0.8s 220621
    else if(init == 0) {buf[9]=speed;buf[10]=0x00; }//speed, position second = 3s
    else if(init == 2) {buf[9]=0;buf[10]=speed; }//speed, position second = 3s
    //buf[10]=0x00;//reservation
    buf[11]=0x00;//reservation
    //printf("%d: DSS %d %d %d\n", osKernelGetTickCount(), speed, buf[9], buf[10]);
    //FF FE 00 06 EC 03 00 00 00 0A
    //0  1  2  3  4  5  6  7  8  9
    for(int i=2;i<SERVO_BUFLEN;i++) {checksum_val += buf[i];}//checksum ~(Packet 2 + Packet 3 + Packet 5 + … + Packet N) [1byte]
    buf[4]=~(checksum_val);//checksum ~(Packet 2 + Packet 3 + Packet 5 + … + Packet N) [1byte]
    checksum_val=0x00;//checksum

    memcpy(str+(12*id), buf, sizeof(buf));

}

uint32_t ServoMotor_read()
{

	uint8_t cur_val = 0;
    char checksum_tmp = 0;
    uint8_t buf[12];
    uint8_t tx_buf[6]={0xFF,0xFE,0x00,0x02,0x5B,0xA2};//header,header,id fixed,length,checksum,speed feedback request
    uint32_t deg_val = 0;
    int rev_msg = 0;

	Read_flag = 1;
	ServoMotor_write(tx_buf);
	Read_flag = 0;

#if 1
	HAL_Delay(5);//need delay time about 14ms

	if(flag_rx == 0) {
		printf("tmp_rx: ");
		for(int i=0;i<12;i++){printf("%02X ", tmp_rx[i]);}
		printf("\n ");

		rev_msg = HAL_UART_Receive_IT(&huart3, tmp_rx, 12);
		if(rev_msg != 0){printf("0=ok, 1=err, 2=busy uart3 rev: %d\n", rev_msg);}
		else{
			printf("uart3 rev error\n");
			return 0xff;
		}    //interrupt error
	}


#else
//	printf("hal_rev: %d\n", HAL_UART_Receive_IT(&huart3, tmp_rx, 12));
//	HAL_Delay(10);

	while(1){
		if(flag_rx == 1) {
			printf("test while\n");
			HAL_Delay(10);
			break;
		}
		printf("noo receive!!\n");

		HAL_Delay(10);
		printf("hal_rev while: %d\n", HAL_UART_Receive_IT(&huart3, tmp_rx, 12));
	}
	if(flag_rx == 0) {
		printf("tmp_rx: ");
		for(int i=0;i<12;i++){printf("%02X ", tmp_rx[i]);}
		printf("\n ");
		return 0xff;    //interrupt error
	}
#endif

	else
	{
		flag_rx = 0;

		for(int i=0;i<12;i++){
			if(tmp_rx[i]==0xFF && tmp_rx[i+1]==0xFE)//parsing
			{
				for(int j=0;j<12;j++)
				{
					if(i+j<12){buf[j]=tmp_rx[i+j];}
					else {buf[j]=tmp_rx[i+j-12];}
				}

				printf("buf: ");
				for(int i=0;i<12;i++){printf("%02X ", buf[i]);}
				printf("\n ");

				checksum_val = buf[2]+buf[3];//buf[2]+buf[3]= id+length
				printf("buf[3] : %d\n", buf[3]);

				for(int i=5;i<buf[3]+4;i++) {
					printf("count i, checksum_val: %d, %02X\n", i, checksum_val);
					checksum_val += buf[i];

				}//checksum ~(Packet 2 + Packet 3 + Packet '5' + … + Packet N) [1byte]
				checksum_tmp = ~(checksum_val);
				checksum_val = 0;
				if(buf[4]==checksum_tmp)
				{
					cur_val=buf[11];
					//deg_val=(65535 - ((buf[9]*256) + buf[10])) / 10;//(65535-(degree_hi*256+degree_low))/degree unit=10~90, degree unit=0.1
					deg_val=(65535 - ((buf[9]*256) + buf[10]));
					//printf("deg: %d %d\n", buf[9],  buf[10]);

				}//BMC_buf[11]=current, unit 1 = 100mA
				//printf("here is servo: %d, %d", buf[9], buf[10]);
				//printf("hihi%d\n", buf[11]);
				else {
					printf("before buf: ");
					for(int i=0;i<12;i++){printf("%02X ", buf[i]);}
					printf("\n ");

					printf("checksum error buf[4] %02X == %02X checksum_tmp \n", buf[4], checksum_tmp);

					for(int i=0;i<SERVO_BUFLEN;i++) {buf[i]=0;}//buf init
					for(int i=0;i<12;i++) {tmp_rx[i]=0;}//tmp_rx init
					return 0xfe;    //interrupt error
				}


				break;
			}
		}
	}
	//else {return 0xff;}

	for(int i=0;i<SERVO_BUFLEN;i++) {buf[i]=0;}//buf init
	for(int i=0;i<12;i++) {tmp_rx[i]=0;}//tmp_rx init

    return cur_val;
    //return deg_val;
//	printf("deg%d\n", deg_val);
	//return deg_val;

}
