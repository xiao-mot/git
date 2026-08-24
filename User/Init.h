#ifndef __Init_H
#define __Init_H
#include "stm32f10x.h"               
#include "all_data.h"
#include "OLED.h"
#include "SERIAL.h"
#include "KEY.h"
#include "String_To_Num.h"
#include "Delay.h"
#include "Encoder.h"
#include "Motor.h "
#include "Pid.h"
#include "PWM.h"
#include "Timer4.h"
#include "stdio.h"  //Ö§³Öprint 
#include "mpu6050.h" 
#include "servo.h"
#include "control.h"
#include "Filter.h"
#include "step_diver.h"
#include "oled.h"
#include "HWT101uart.h"

void ALL_Init(void);
void pid_Para_Init_Revolve(void);
void pid_Para_Init_Location(int mode);
void dainjiInit(void);



#endif

