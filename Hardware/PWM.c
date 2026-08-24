#include "stm32f10x.h"                  // Device header


//void TIM2_PWM_Config(void)
//{
//	
//	  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//  TIM_OCInitTypeDef TIM_OCInitStructure;
//  GPIO_InitTypeDef GPIO_InitStructure;
//  /* 使能GPIOA时钟时钟 */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  /* 使能定时器2时钟 */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = 999; //定时器计数周期 0-999  1000	
//  TIM_TimeBaseStructure.TIM_Prescaler = 9; //设置预分频：8+1分频   8K PWM频率
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分频系数：不分频
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//多加了一句
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式

//  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

//  /* PWM1 Mode configuration: Channel */
//  TIM_OCStructInit(&TIM_OCInitStructure);//多加了一句
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //配置为PWM模式1
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = 0;
//    //设置跳变值，当计数器计数到这个值时，电平发生跳变(即占空比) 初始值0
////  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
////    //当定时器计数值小于定时设定值时为高电平
////  /* 使能通道1 */
////  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
////  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
////  /* 使能通道2 */
////  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
////  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
////  /* 使能通道3 */
////  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
////  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
////  /* 使能通道4 */
////  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
////  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

////  TIM_ARRPreloadConfig(TIM2, ENABLE); // 使能TIM2重载寄存器ARR
//  TIM_Cmd(TIM2, ENABLE); //使能定时器2
//}




void PWM_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM8);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;//2000-1;    //100 - 1;	 80	//ARR//pwm f	6k//原始校赛1000
	TIM_TimeBaseInitStructure.TIM_Prescaler = 12 - 1;   //2-1;    //36 - 1;	 45	//PSC//
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	TIM_Cmd(TIM8, ENABLE);
}
//
void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM8, Compare);
}

void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM8, Compare);
}

void PWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare3(TIM8, Compare);
}

void PWM_SetCompare4(uint16_t Compare)
{
	TIM_SetCompare4(TIM8, Compare);
}
