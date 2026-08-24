#include "stm32f10x.h"                
#include "PWM.h"

#define left_front_wheel_GPIO      GPIOC
#define left_front_wheel_PIN1      GPIO_Pin_0
#define left_front_wheel_PIN2      GPIO_Pin_1

#define right_front_wheel_GPIO     GPIOB
#define right_front_wheel_PIN1     GPIO_Pin_12
#define right_front_wheel_PIN2     GPIO_Pin_13

#define left_back_wheel_GPIO       GPIOC
#define left_back_wheel_PIN1       GPIO_Pin_2
#define left_back_wheel_PIN2       GPIO_Pin_3                                         

#define right_back_wheel_GPIO      GPIOB
#define right_back_wheel_PIN1      GPIO_Pin_15
#define right_back_wheel_PIN2      GPIO_Pin_14

void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = left_front_wheel_PIN1 | left_front_wheel_PIN2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(left_front_wheel_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = right_front_wheel_PIN1 | right_front_wheel_PIN2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(right_front_wheel_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = left_back_wheel_PIN1 | left_back_wheel_PIN2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(left_back_wheel_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = right_back_wheel_PIN1 | right_back_wheel_PIN2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(right_back_wheel_GPIO, &GPIO_InitStructure);
	
	PWM_Init();
}


void Motor_left_front_wheel_SetSpeed(int16_t lf_speed)//左前电机正反转
{
	if(lf_speed >= 0)
	{
		GPIO_ResetBits(left_front_wheel_GPIO,left_front_wheel_PIN1);
		GPIO_SetBits(left_front_wheel_GPIO,left_front_wheel_PIN2);
		PWM_SetCompare1(lf_speed);	
	}
	else
	{
		GPIO_SetBits(left_front_wheel_GPIO,left_front_wheel_PIN1);
		GPIO_ResetBits(left_front_wheel_GPIO,left_front_wheel_PIN2);
	    
		PWM_SetCompare1(-lf_speed);
		
	}

}



void Motor_right_front_wheel_SetSpeed(int16_t rfront_speed)//右前电机正反转
{
	if (rfront_speed >= 0)
	{
		GPIO_ResetBits(right_front_wheel_GPIO, right_front_wheel_PIN2);
		GPIO_SetBits(right_front_wheel_GPIO, right_front_wheel_PIN1);
		PWM_SetCompare3(rfront_speed);
	}
	else
	{
		GPIO_SetBits(right_front_wheel_GPIO, right_front_wheel_PIN2);
		GPIO_ResetBits(right_front_wheel_GPIO, right_front_wheel_PIN1);
		PWM_SetCompare3(-rfront_speed);
	}
}



void Motor_left_back_wheel_SetSpeed(int16_t lb_speed)//左后电机正反转
{
	if (lb_speed>= 0)
	{
		GPIO_ResetBits(left_back_wheel_GPIO , left_back_wheel_PIN2);
		GPIO_SetBits(left_back_wheel_GPIO , left_back_wheel_PIN1);
		PWM_SetCompare2(lb_speed);
	}
	else
	{
		GPIO_SetBits(left_back_wheel_GPIO , left_back_wheel_PIN2);
		GPIO_ResetBits(left_back_wheel_GPIO ,left_back_wheel_PIN1);
		PWM_SetCompare2(-lb_speed);
	}
}



void Motor_right_back_wheel_SetSpeed(int16_t rb_speed)//右后电机正反转
{
	if (rb_speed >= 0)
	{
		GPIO_ResetBits(right_back_wheel_GPIO, right_back_wheel_PIN2);
		GPIO_SetBits(right_back_wheel_GPIO, right_back_wheel_PIN1);
		PWM_SetCompare4(rb_speed);
	}
	else
	{
		GPIO_SetBits(right_back_wheel_GPIO, right_back_wheel_PIN2);
		GPIO_ResetBits(right_back_wheel_GPIO, right_back_wheel_PIN1);
		PWM_SetCompare4(-rb_speed);
	}
}
