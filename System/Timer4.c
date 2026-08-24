#include "String_To_Num.h"
#include "Delay.h"
#include "Encoder.h"
#include "Pid.h"
#include "PWM.h"
#include "Timer4.h"
#include "stdio.h" 
#include "mpu6050.h" 
#include "init.h"
#include "all_data.h"
#include "control.h"

// ��ʱ��6���жϴ�������
void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // ʹ��TIM6ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    // ���ü�ʱ����ʱ�ӷ�Ƶ
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1; //����Ԥ��Ƶֵ, 10kHz�ļ���Ƶ��

    // ���ü�ʱ��������ֵ�ͼ�ʱ��λ
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;      //�����Զ���װ��ֵ, ʵ��1ms�ļ�ʱ
    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    // ���ü�ʱ��6
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    // ʹ�ܼ�ʱ��6�ĸ����ж�
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    // ���ü�ʱ��6���ж�
    NVIC_EnableIRQ(TIM6_IRQn);

    NVIC_InitTypeDef  NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel   =TIM6_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
    NVIC_InitStruct.NVIC_IRQChannelCmd  =ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // ʹ�ܶ�ʱ��6
    TIM_Cmd(TIM6,ENABLE);
}




void TIM6_IRQHandler(void)//1ms
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{	
		static unsigned char i,j;//,k;
		
		i++;i%=5;  //����?
        j++;j%=300;	
//        k++;k%=30;
		
        
		if(i==0)
		{
			MpuGetData();
			GetAngle(&MPU6050,&Angle,0.005f);//0.005f   //MPU6050 yaw
			//Angle.yaw=global_angle;           //HWT101 yaw (����,�ɹ�������)
			//MPU6050.gyroZ=angular_velocity_z; //HWT101 gyroZ (����,�ɹ�������)
		}
		else if(i == 1)
		{	
			Get_Speed();
            //Get_Dis();
		}
		else if(i == 2)
		{
			//Control_Mode();
            //Mode_Flag = REVOLVE_MODE_150 ; 
            //LineMove_Mode();
		    //Location_State();
			//Get_wheel_circle(0.9); 
		}
		else if(i == 3)
		{
			Pid_Control_Trans();
		}
		
		
		if(j == 4)
		{
			Serial_SendPacket();//����λ����ͷ ���Ͷ�ά�������Լ��Ӿ�����
		}
		
		
//	    if(k == 15)
//        {
//			Serial2_SendByte(Write_Flag);//�������ֲ鿴��ȥdebug
//        }
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}

/*
void TIM6_IRQHandler(void)//1ms
{
        //ԭ�����? 
		i++;
		j++;
		i%=5;
		j%=100;	  
		if(i==0)
		{
			MpuGetData();
			GetAngle(&MPU6050,&Angle,0.005f);
		}
		if(i == 1)
		{
			Get_Speed();
		}
		if(i==2)
		{
			Control_Mode();//�����ܴ���  ��������ָ���Լ��ñ�־λ
		}
		if(i == 3)
		{
			Pid_Control_Trans();//ִ������ָ�ģʽ��������
		}
		if(j == 3)
		{
			Serial_SendPacket();//����λ����ͷ ���Ͷ�ά�������Լ��Ӿ�����
		}
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
  }
}
*/



