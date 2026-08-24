#ifndef __ALL_DATA_H
#define __ALL_DATA_H

#include "stdint.h"

typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}_st_AngE;


typedef volatile struct
{
	float desired;       //< set point
	float offset;        //
	float prevError;     //< previous error
	float integ;         //< integral
	float kp;            //< proportional gain
	float ki;            //< integral gain
	float kd;            //< derivative gain
	float IntegLimitHigh;//< integral limit
	float IntegLimitLow;
	float measured;
	float out;
	float OutLimitHigh;
	float OutLimitLow;
}PidObject;


typedef struct
{
    float lastP;		//�ϴε�Э����
    float nowP;			//���ε�Э����
    float x_hat;		//�������˲��ļ���ֵ����Ϊ��������ֵ
    float Kg;			//����������ϵ��
    float Q;			//��������
    float R;			//��������
}Kalman;



//�ⲿ���ļ����趨�����±���
extern PidObject PID_FOR_L,PID_FOR_R,PID_BAC_L,PID_BAC_R;
extern float F_L_Speed,F_R_Speed,B_L_Speed,B_R_Speed;
extern PidObject pidYaw;
extern PidObject pidRateZ;
extern PidObject pidLX;
extern PidObject pidLY;


extern _st_Mpu MPU6050;
extern _st_AngE Angle;
extern unsigned char Mode_Flag;//���з�ʽ��־λ

extern const float Gyro_G;//�����ǳ�ʼ������+ -2000��ÿ����1 / (65536 / 4000) = 0.03051756*2	

extern float LX,LY,Angle_Yaw;
extern unsigned char Serial_TxPacket[10];//�������ݰ�
extern unsigned char Write_Flag;         //���ڴ��������
extern unsigned char Yellow_Flag;        //ûɶ��
extern PidObject *(pPidObject[]); 
extern unsigned char Catch_Flag;         //ץȡģʽ��־λ
extern Kalman KF_X,KF_Y,KF_FL,KF_FR,KF_BL,KF_BR;

extern uint8_t Serial_TxPacket[10];

extern uint8_t Serial5_RxPacket[10];
extern unsigned char QrCode;
extern Kalman KF_Angle;


//Ѱ�ף�Ѱ��ɫ��־λ
extern unsigned char Serial_Yaw;

extern unsigned char Place_Red;
extern unsigned char Place_Blue;
extern unsigned char Place_Green;

extern unsigned char Catch_Red;
extern unsigned char Catch_Green;
extern unsigned char Catch_Blue;

extern float Last_Yaw;
//HWT101
extern volatile float global_angle;
extern volatile uint8_t new_data_received;
extern volatile float angular_velocity_y;//���ٶ�y
extern volatile float angular_velocity_z;//���ٶ�z
extern uint8_t received_data_packet[11];


extern unsigned char BuzzerFlag;



#endif
