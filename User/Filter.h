#ifndef __FILTER_H
#define __FILTER_H
#include "stm32f10x.h"                  // Device header
#include "all_data.h"


float Kalman_Filter(Kalman *KF, float input);
void Kalman_Init(void);




#endif
