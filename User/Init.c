#include "init.h"

PidObject PID_FOR_L,PID_FOR_R,PID_BAC_L,PID_BAC_R;
PidObject pidYaw;
PidObject pidRateZ;
PidObject pidLX;
PidObject pidLY;


void ALL_Init()
{
	/*NVIC�жϷ���*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	pid_Para_Init_Location(1);
	Kalman_Init();
	CamCtrl_Init(&cam_ctrl);
	debugPinInit();
    
	Motor_Init();
    
	Micorstep_Driver_Init();
	UP(start);Delay_ms(100);

	Servo_Init();
   
	Encoder_Init();
	PWM1_SetCompare4(could_out);Delay_ms(500);//��̨��

	Serial_4_Init();  
    Serial_2_Init(); 
	Serial_5_Init();//���ڻ沨�� 
	
// yaw source: MPU6050 (HWT101 not installed)
	IIC_Init();
	MpuInit();	
	MpuGetOffset();//У׼

//	//��������
//	while(1)
//	{
//		if(Write_Flag == 0x02)
//				break;
//	}

//	Usart3_Init();//HWT101
	Delay_ms(50);
//	Usart3_Init_2();//HWT101


	TIM6_Init();
}


void pid_Para_Init_Location(int mode)
{

	PID_FOR_L.kp = 16;  
	PID_FOR_L.ki =3;    
	PID_FOR_L.kd = 0.05;
	
	PID_FOR_R.kp = 16;
	PID_FOR_R.ki =3;	
	PID_FOR_R.kd = 0.05;
	
	PID_BAC_L.kp = 16;
	PID_BAC_L.ki =3;	
	PID_BAC_L.kd = 0.05;	
	
	PID_BAC_R.kp = 16;
	PID_BAC_R.ki =3;
	PID_BAC_R.kd = 0.05;
	
	pidYaw.kp = 3.286;//0.65
	pidYaw.ki = 0.39;//0.08;//0.04;//0.015	
	pidYaw.kd = 0.34;//0.01	
	
	pidRateZ.kp = 0.80;//0.105
	pidRateZ.ki = 0.21;	//0.25
	pidRateZ.kd = 0.00111;//0.00101
	
	if(mode == 1)
	{
		//������ò���
		pidLX.kp =  0.042;//0.050;// 0.065;  
		pidLX.ki =  0;
		pidLX.kd =  0.027;
		
		pidLY.kp = 0.1;//0.055;
		pidLY.ki = 0;	  
		pidLY.kd = 0;	
	}
	else if(mode == 2)
	{
		//Բ����ò���
		pidLX.kp =  0.06;//0.050;// 0.065;  
		pidLX.ki =  0.0052;//0.004;//0.016;	
		pidLX.kd =  0.102;//0.026;//0.096;//0.021;
		
		pidLY.kp = 0.26;//0.066;//0.055;
		pidLY.ki = 0.009;//0.0059;	  
		pidLY.kd = 17.949989;//0.007;//0.007;
		//У��12.0
//		pidLX.kp =  0.06;//0.050;// 0.065;  
//		pidLX.ki =  0.004;//0.004;//0.016;	
//		pidLX.kd =  0.102;//0.026;//0.096;//0.021;
//		
//		pidLY.kp = 0.26;//0.066;//0.055;
//		pidLY.ki = 0.006;//0.0059;	  
//		pidLY.kd = 17.931;//0.007;//0.007;	
	}

}

/*//��ӡ����������
void dainjiInit()
{
    GPIO_InitTypeDef DAINJI_Structure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	
    DAINJI_Structure.GPIO_Mode = GPIO_Mode_Out_PP;
    DAINJI_Structure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	DAINJI_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&DAINJI_Structure);
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_5 | GPIO_Pin_4);
	
	DAINJI_Structure.GPIO_Mode = GPIO_Mode_Out_PP;
    DAINJI_Structure.GPIO_Pin = GPIO_Pin_12;
	DAINJI_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&DAINJI_Structure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_12);
	
	Delay_ms(1000);
}
*/
