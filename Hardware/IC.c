#include "stm32f10x.h"                  // Device header

void IC_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	
	TIM_Cmd(TIM3, ENABLE);
}

void TIM_SetCCR(TIM_TypeDef* TIMx, uint16_t Counter)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  /* Set the Counter Register value */
  TIMx->CCR1 = Counter;
}

uint32_t IC_GetEncode(void)
{
	uint32_t Temp;
	Temp = 72000000/TIM_GetCapture1(TIM3);
	TIM_SetCCR(TIM3, 0);
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 0 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 0 )
	{
		Temp = 0;
	}
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 1 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 0 )
	{
		Temp = Temp ;
	}
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 0 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 1 )
	{
		Temp = -Temp;
	}
	return Temp;
}
