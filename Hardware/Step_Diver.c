#include "step_diver.h"

//#include "data_def.h"
//此编码器为共阴极接法
#define ENA_GPIO   GPIOB  
#define ENA        GPIO_Pin_4
#define DIR_GPIO   GPIOB
#define DIR        GPIO_Pin_5
#define PUL_GPIO   GPIOA              //STP，脉冲信号输入
#define PUL        GPIO_Pin_12

uint8_t  PUL_state , turn , Micorstep;//中断生成脉冲标志位     开关定时器标志位     软件使能上电能自锁
float    Tim;                         //脉冲数的两倍或定时时间
float    Number_Of_Turns;             //步进电机转动圈数


void Micorstep_Driver_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = ENA;
 	GPIO_Init(ENA_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = DIR;
 	GPIO_Init(DIR_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PUL;
 	GPIO_Init(PUL_GPIO, &GPIO_InitStructure);

    
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period     =800-1;   //550//1000//800最好
    TIM_TimeBaseInitStruct.TIM_Prescaler  =45-1;    //为 72 时定时1ms  45
    TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStruct);

	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
//  TIM_ITConfig(TIM7, TIM_IT_Update,ENABLE);//开启中断
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_InitTypeDef  NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel   =TIM7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =2;//优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
    NVIC_InitStruct.NVIC_IRQChannelCmd  =ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM7,ENABLE);
	
}

void Micorstep_Enable(void)
{
	Micorstep = 0;
}

void UP (float number_of_turns)
{  
	while(Micorstep == 0)
	{
		Number_Of_Turns = number_of_turns;
		if(turn == 0)
		{
			GPIO_SetBits(ENA_GPIO,ENA);
			GPIO_ResetBits(DIR_GPIO,DIR);//上升
			
			TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
			NVIC_EnableIRQ(TIM7_IRQn);
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_Init(&NVIC_InitStructure);
			
			TIM_Cmd(TIM7,ENABLE);
		}
		else if(turn == 1)
		{
			Tim = 0; 
			
			TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
			NVIC_DisableIRQ(TIM7_IRQn);
			
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_Init(&NVIC_InitStructure);
			
			TIM_Cmd(TIM7, DISABLE);
			TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
			Micorstep = 1;//每次调用UP和DOWN函数时，都要使能一下
			turn = 0;
		}
	}
}

void DOWN(float number_of_turns)
{  
	while(Micorstep == 0)
	{
		Number_Of_Turns = number_of_turns;
		if(turn == 0)
		{
			GPIO_SetBits(ENA_GPIO,ENA);
			GPIO_SetBits(DIR_GPIO,DIR);//下降*
			
			TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
			NVIC_EnableIRQ(TIM7_IRQn);
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_Init(&NVIC_InitStructure);
			
			TIM_Cmd(TIM7,ENABLE);
		}
		else if(turn == 1)
		{
			Tim = 0; 
			
			TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
			NVIC_DisableIRQ(TIM7_IRQn);
			
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_Init(&NVIC_InitStructure);
			
			TIM_Cmd(TIM7, DISABLE);
			TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
			Micorstep = 1;//每次调用UP和DOWN函数时，都要使能一下
			turn = 0;
		}
	}
}

void TIM7_IRQHandler(void)//负责改变turn的值，以及生成矩形脉冲PUL
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		if (PUL_state == 1)
		{
			GPIO_SetBits(PUL_GPIO,PUL);
			PUL_state=0;
			Tim++;
		}
	    else if(PUL_state == 0)
		{
			GPIO_ResetBits(PUL_GPIO,PUL);
			PUL_state=1;
			Tim++;
		}		
		if(Tim >= Number_Of_Turns*1598)//编码器默认16细分，即3200个脉冲数为一圈转
		{
			Tim = 0;
			turn = 1;	
		} 			
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}
