#include "stm32f10x.h"                  // Device header
#include "My_PID.h"  
//extern float Yaw_Value;

float Yaw_Out;

//typedef struct
//{  
//    float desire;//期望值
//	float measured_value;//测量值
//	float integral;//积分
//	float derivative;//微分 
//  float kp;           //< proportional gain
//	float ki;           //< integral gain
//	float kd;           //< derivative gain
//	float IntegLimitHigh;       //< integral limit
//	float IntegLimitLow;
//	float OutLimitHigh;//输出限幅
//	float OutLimitLow;
//	float error0;
//	float error1;
//	float out;
//}PID_Structure;



extern float ypr[3];


PID_Structure PidYaw;

void PID_ParamInit(void)
{

	PidYaw.kp = 15.0f;
	PidYaw.ki = 1.5f;
	PidYaw.kd = 1.5f;	
}

void PID_Date_Init(PID_Structure* pid)
{
	pid->desire=0;
	pid->error0=0;
	pid->derivative=0;
	pid->error1=0;
	pid->measured_value=0;
	pid->out=0;
	pid->integral=0;
	
}


void PID_Limt_Init()
{
	PidYaw.OutLimitHigh=120;
	PidYaw.OutLimitLow=-120;
}

void pid_update(void)
{
	PidYaw.desire=0;
	PidYaw.measured_value =0; //Yaw_Value;	
}

void PID_Control(PID_Structure* pid)
{
  pid_update();
	pid->error0=pid->desire-pid->measured_value;//本次偏差
  // if(pid->error0<1 && pid->error0>-1)
  // {
  //   pid->error0 = 0;
  // }
	if(pid->error0>2||pid->error0<-2)
	{
	pid->integral=pid->integral+pid->error0;//积分
	}
	if(pid->integral > pid->IntegLimitHigh)
	{
		pid->integral=pid->IntegLimitHigh;//积分限幅
	}
		if(pid->integral < pid->IntegLimitLow)
	{
		pid->integral=pid->IntegLimitLow;//积分限幅
	}
	pid->derivative=(pid->error0-pid->error1);
	pid->out=pid->kp*pid->error0+pid->ki*pid->integral*0.05+pid->kd*pid->derivative/0.05;
	if(pid->out>pid->OutLimitHigh)
	{
		pid->out=pid->OutLimitHigh;//输出限幅
	}
	if(pid->out<pid->OutLimitLow)
	{
		pid->out=pid->OutLimitLow;
    
	}
    Yaw_Out = pid->out;
	pid->error1=pid->error0;
}
