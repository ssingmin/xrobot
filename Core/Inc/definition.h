/*
 * definition.h
 *
 *  Created on: Jan 26, 2023
 *      Author: sangmin_lee
 */

#ifndef INC_DEFINITION_H_
#define INC_DEFINITION_H_

#include "can.h"

enum XbotRW {
	Xbot_R = 0,
	Xbot_W = 1,
};

#define ENDMODETIME 2000

#define STMotorID1 0
#define STMotorID2 1
#define STMotorID3 2
#define STMotorID4 3

#define RPM_1 20
#define RPM_2 20

#define PERIOD_STATUS_LED 500U
//#define PERIOD_CANCOMM 100U
#define PERIOD_CANCOMM 10U
#define PERIOD_STEERING 50U
//#define PERIOD_STEERING 500U
#define PERIOD_NP_LED 500U
#define PERIOD_FAN 500U
#define PERIOD_IRQ_PSx 10U

#define NUM_NPLED 29

//unit is mm
#define MATH_PI 3.14159265358979323846
#define SIGNIFICANT_FIGURES 1000//NOT USED
#define WHEEL_RADIUS 86.5
#define WHEEL_TREAD 419
#define WHEEL_BASE 460

#define CONSTANT_VEL2 9.05825881785//(2*MATH_PI*WHEEL_RADIUS)/60
#define CONSTANT_VEL 0.11039649231807768955076370779228	//60/(2*MATH_PI*WHEEL_RADIUS)
#define CONSTANT_ANGLE 311.11131448406	//(sqrt(W^2+L^2))/2
#define CONSTANT_C_AxC_V 34.345597839506582021710853238711	//CONSTANT_ANGLE * CONSTANT_VEL
#define ANGLE_RAD atan2(Tar_cmd_v_y,Tar_cmd_v_x)
#define ANGLE_VEL 0.7387880327864079	//atan2(WHEEL_TREAD,WHEEL_BASE)


#define ENC_RESOLUTION 4096
#define POLE_PAIR 10
#define MAX_RPM 200
#define BREAK_OPTION true
#define RATED_CURRENT 8
#define MAX_CURRENT 16
#define HALL_OFFSET 240
#define KP_GAIN 500 //default 500, manual 550
#define KI_GAIN 100 //default 100, manual 110


#endif /* INC_DEFINITION_H_ */
