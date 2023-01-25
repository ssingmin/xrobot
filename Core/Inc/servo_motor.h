#ifndef __SERVO_MOTOR_H__
#define __SERVO_MOTOR_H__
#include "main.h"
#include "stm32f4xx_hal.h"

#define SERVO_BUFLEN  10  //max length of nurirobot protocol

#define SERVO_ORIGIN_VAL 1050 // 0->open, 2000->close, resolution 0.01 degree
#define SERVO_OPEN  1,9000-SERVO_ORIGIN_VAL,0  //direction= ccw, position 90 degree, normal
#define SERVO_INITOPEN  1,3500,0  //direction= ccw, position 90 degree, normal
#define SERVO_CLOSING  1,3500-SERVO_ORIGIN_VAL,0  //direction= ccw, position 20 degree, normal
#define SERVO_CLOSE  0,SERVO_ORIGIN_VAL,0  //direction= ccw, position 10 degree, normal
#define SERVO_INIT_START  0,25,1  //direction= cw, speed, init mode
#define SERVO_INIT_STOP  0,0,1  //direction= cw, speed, init mode
#define SERVO_ORIGIN  0,0,0  //direction= cw, speed, init mode

//don't use//
#define SERVO_INIT_OPEN  1,1000,0  //direction= cw, speed, init mode

#define SERVO_CLOSING_TIME  200

#define SERVO_WARNING_OPEN  1,300,0  //direction= ccw, position 3 degree, normal //when closing, open 3degree
#define SERVO_WARNING_CLOSE  0,300,0  //direction= ccw, position 3 degree, normal //when opening, close 3degree
/////////////

#define STOP_SPEED 10  //stop speed * 0.1,  // 0.3, 0.5, 0.7
extern char checksum_val;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void baudrate_reinit();
void ServoMotor_write(const char* str);
void ServoMotor_control(char direction, unsigned short position, char init);
uint32_t ServoMotor_read();

#endif
