#include "Servo.h"

int Grab_falg=0;
unsigned int claw_poriect_free = 700;//���ڲ�����׼����ֵ���лع龫׼ֵ

void Servo_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM1);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;//PWMƵ�� = 72M/72/20000 =50hz
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;//ռ�ձ�=CCR/ARR 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	//��������Ƚ�OC
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	
	TIM_OCInitStructure.TIM_Pulse = claw_free; //A9צ��      arr 600-1050
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = stage_1;   //A10����̨   arr��̨ �۲�800  ����2000
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = could_in;  //A11��̨     arr��̨ 1200 ��׼ ����1 2 3 2200 1500 800 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
    TIM_CtrlPWMOutputs(TIM1,ENABLE);
//	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_Cmd(TIM1, ENABLE);
}


void PWM1_SetCompare2(uint16_t Compare)//PA9��еצ
{
	TIM_SetCompare2(TIM1, Compare);
}

void PWM1_SetCompare3(uint16_t Compare)//PA10����̨
{
	TIM_SetCompare3(TIM1, Compare);
}

void PWM1_SetCompare4(uint16_t Compare)//PA11��̨
{
	TIM_SetCompare4(TIM1, Compare);
}


void cloud_tai(int site) // 770 1950
{
     static int now_site=could_in; ////
	 if(now_site<site)//��̨�ӹ۲�λ�ã��Ƕ�С��ת������̨λ�ã��Ƕȴ�
	 {	 
		for(;now_site<site;now_site=now_site+10)
		{
			PWM1_SetCompare4(now_site);
			Delay_ms(20);
		}
	
	 }
    else if(now_site>site)//��̨������̨λ�ã��Ƕȴ�ת���۲�λ�ã��Ƕ�С��
	{
		
        for(;now_site>site;now_site=now_site-10)
    	{
			PWM1_SetCompare4(now_site);
			Delay_ms(20);
	   	}
	}
		
	now_site=site;
	PWM1_SetCompare4(now_site);
	Delay_ms(50); //���ڲ�����׼����ֵ���лع龫׼ֵ
}



void robotic_grab(int site) //890 760 500 claw_grabץȡ claw_free�ͷ�  claw_look��ʼλ�÷���openmv�۲�
{
    static int now_site=claw_grab;///
	 if(now_site<site)
	 {	 
	 	 for(;now_site<site;now_site=now_site+10)//��еצ���ͷ�״̬��С�Ƕȣ�תΪץȡ״̬����Ƕȣ�
		{
		
		    PWM1_SetCompare2(now_site);
		    Delay_ms(20);
		}
			PWM1_SetCompare2(claw_poriect_free);
	        Delay_ms(50); //���ڲ�����׼����ֵ���лع龫׼ֵ	
	 }
    else if(now_site>site) //
	{
		
		for(;now_site>site;now_site=now_site-10)
		{
		   PWM1_SetCompare2(now_site);
		   Delay_ms(20);
		}
	}
    now_site=claw_grab;
	
	PWM1_SetCompare2(500);	
	Delay_ms(1000);
}


