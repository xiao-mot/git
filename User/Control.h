#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f10x.h"                  
#include "ALL_DATA.h"
#include "Motor.h"
#include "PID.h"
#include "Delay.h"
#include "Servo.h"
#include "Step_Diver.h"
#include "Serial.h"
#include "Encoder.h"
#include "Init.h"
#include "cam_center.h"

void Forward_Mode(float dt);
void Forward_Mode_Low(float dt);
void Back_Mode(float dt);
void Back_Mode_Low(float dt);
void Stop_Mode(void);
void Trans_Mode(float dt);     
void Trans_Left_Mode_Run(float dt);
void Revolve_Mode(float dt); 
void Revolve_Mode_150(float dt);
void Revolve_Mode_0(float dt);


void Location_Mode(float dt);     
void Location_Mode_Place(float dt);
void Location_State(float Designed_LX,float Designed_LY,uint8_t erro);


void Place_Mode(unsigned char Color);
void Place_Sthing(void);
void Place_ALL(unsigned char Place_F);
void Place_ALL_consient(unsigned char Place_F);

void Catch_Mode(unsigned char Color); 
void Catch_All(unsigned char Catch_F);
void Catch_Sthing(void);              


void Control_Mode(void);     
void Pid_Control_Trans(void);

void LineMove_Mode(void);
void Control_Moto(void);

//**********************************************************************
//Pid_Control_Trans�õ�
#define STOP_MODE           255
#define REVOLVE_MODE        12
#define FOR_MODE            13
#define TRANS_MODE          14
#define LOCATION_MODE       15
#define TRANS_LEFT_MODE     16
#define BACK_MODE           17
#define LOCATION_PLACE_MODE 18
#define FOR_MODE_LOW        19
#define REVOLVE_MODE_150    20
#define REVOLVE_MODE_0      21
#define BACK_MODE_Low       22



#define start_1 1
//#define start_2 2
//#define start_3 3
//#define start_4 4
//#define start_5 5
//#define start_6 6
//#define start_7 7
//#define start_8 8
//#define start_9 9



#define MOVE_10 10
#define MOVE_11 11
#define MOVE_12 12
#define MOVE_13 13
#define MOVE_14 14
#define MOVE_15 15
#define MOVE_16 16
#define MOVE_17 17
#define MOVE_18 18
#define MOVE_19 19
#define MOVE_20 20
#define MOVE_21 21
#define MOVE_22 22
#define MOVE_23 23
#define MOVE_24 24
#define MOVE_25 25
#define MOVE_26 26
#define MOVE_27 27
#define MOVE_28 28
#define MOVE_29 29
#define MOVE_30 30


//
#define WAIT_50 50
#define WAIT_51 51
#define WAIT_52 52
#define WAIT_53 53
#define WAIT_54 54
#define WAIT_55 55
#define WAIT_56 56
#define WAIT_57 57
#define WAIT_58 58
#define WAIT_59 59
#define WAIT_60 60
#define WAIT_61 61

//抓取
#define GRASP_70 70
#define GRASP_71 71
#define GRASP_72 72
#define GRASP_73 73
#define GRASP_74 74
#define GRASP_75 75
#define GRASP_76 76
#define GRASP_77 77
#define GRASP_78 78
#define GRASP_79 79
#define GRASP_80 80
#define GRASP_81 81
#define GRASP_82 82
#define GRASP_83 83




//识别
#define IDENTIFY_90 90
#define IDENTIFY_91 91
#define IDENTIFY_92 92
#define IDENTIFY_93 93
#define IDENTIFY_94 94
#define IDENTIFY_95 95
#define IDENTIFY_96 96
#define IDENTIFY_97 97
#define IDENTIFY_98 98

//**********************************************************************

#endif


