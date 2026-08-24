#include "stm32f10x.h"               
#include <stdio.h>
#include <stdarg.h>
#include "OLED.h"
#include "all_data.h"
#include "Filter.h"
#include "Serial.h"
#include "string.h"
#include "math.h"
#include "control.h"


uint8_t Serial5_RxPacket[10];

/*Serial_TxPacket
{0x00,0x01,0x02,0x03,0x04,0x05,0x06};
*/
uint8_t Serial_TxPacket[10]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t Serial_RxPacket[10];
uint8_t Serial_RxFlag;


float LX=0,LY=0,Angle_Yaw=0;
unsigned char Write_Flag;
unsigned char Serial_Yaw;//mv����ƫ�ƽǶ�
unsigned char Yellow_Flag;//ûɶ��
float Last_Yaw;


//******************************ͨ��************************************

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

//******************************ͨ��************************************

//***********************************����5***********************************/

void Serial_5_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(UART5, &USART_InitStructure);
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(UART5, ENABLE);
	
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(UART5, Byte);
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
}

void Serial5_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

//int fputc(int ch, FILE *f)
//{
//	Serial_SendByte(ch);
//	return ch;
//}



void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}


void UART5_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(UART5);
		if (RxState == 0)
		{
			if (RxData == 0xFF)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData != 0xFE)
			{
				Serial5_RxPacket[pRxPacket] = RxData;
				pRxPacket ++;
			}
			if (RxData == 0xFE)
			{
				RxState = 0;
				Serial_TxPacket[1] =  Serial5_RxPacket[0];//������5��ά����Ϣͨ������4������еצmv��def Find_color_in_sequence()�����л����
				Serial_TxPacket[2] =  Serial5_RxPacket[1];
				Serial_TxPacket[3] =  Serial5_RxPacket[2];
				Serial_TxPacket[4] =  Serial5_RxPacket[3];//�ڶ�Ȧ������
				Serial_TxPacket[5] =  Serial5_RxPacket[4];
				Serial_TxPacket[6] =  Serial5_RxPacket[5];                
				
                debugPinTurn();
               
			}		
		}
		
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
}


//***********************************����5***********************************/





//*******************************����4***************************************/
#define BUFFER_SIZE_U4 256
uint8_t rxBuffer_U4[BUFFER_SIZE_U4];


void Serial_4_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;


    // 1. ���� GPIOC��UART4 �� DMA2 ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);  // GPIOC ʱ��ʹ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);  // UART4 ʱ��ʹ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);  // DMA2 ʱ��ʹ��

    // 2. ���� GPIOC10��TX���� GPIOC11��RX��Ϊ���ù���
    // ���� TX ���ţ�PC10��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ���� RX ���ţ�PC11��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // ��������
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 3. ���� UART4
    USART_DeInit(UART4);  // ��λ UART4 ����
    USART_InitStructure.USART_BaudRate = 115200;  // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 1λֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;  // ��У��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // ���պͷ���ģʽ
    USART_Init(UART4, &USART_InitStructure);

    // ���� UART4 �Ŀ����ж�
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);  // �����ж�ʹ��

    // ���� NVIC �ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;  // UART4 �ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // �����ж�
    NVIC_Init(&NVIC_InitStructure);

    // 4. ���� DMA ����
    DMA_DeInit(DMA2_Channel3);                                      // ��λ DMA ����
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;// �����ַ��UART4 ���ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxBuffer_U4;   // �ڴ��ַ���������ݻ�����
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;              // ���ݷ��򣺴����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE_U4;              // ��������С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// �����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         // �ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// �������ݿ��ȣ��ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // �ڴ����ݿ��ȣ��ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                 // ����ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;             // �����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                    // �����ڴ浽�ڴ洫��
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);

    // ���� DMA
    DMA_Cmd(DMA2_Channel3, ENABLE);

    // ���� UART4 �� DMA ��������
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);

    // 5. ���� UART4
    USART_Cmd(UART4, ENABLE);
}





extern Kalman KF_LX;
extern Kalman KF_LY;
void UART4_IRQHandler(void) 
{
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET) 
	{
        USART_ReceiveData(UART4);  // �����ݼĴ�������0�� IDLE ��־
        DMA_Cmd(DMA2_Channel3, DISABLE);  // �ر� DM

        uint16_t dataLength = BUFFER_SIZE_U4 - DMA_GetCurrDataCounter(DMA2_Channel3);  // ������ճ���
        
        // �ڴ˴��������յ������ݣ�rxBuffer ��ǰ dataLength ���ֽڣ�

        if(rxBuffer_U4[0] == 0xFF && rxBuffer_U4[6] == 0xFE)
        {
            
            static float last_lx = 0;
            static float last_ly = 0;
            
            float temp_lx = 0;
            float temp_ly = 0;
            
            Serial_Yaw = rxBuffer_U4[1];//ֱ��ƫ�ƽǶ�
            //Last_Yaw = Serial_Yaw;
            //Serial_Yaw = Kalman_Filter(&KF_Angle,Serial_Yaw);
            Last_Yaw = Kalman_Filter(&KF_Angle,Last_Yaw);
            
            temp_lx = (float)rxBuffer_U4[3] ;			
            temp_ly = (float)rxBuffer_U4[4] ;
            
            Write_Flag = rxBuffer_U4[5];
            
            //if(Write_Flag == 0xcb) debugPinTurn();
            //Yellow_Flag = rxBuffer_U4[6];//ûɶ��
            
            
            //printf("%.2lf,%.2lf\n",temp_lx,temp_ly);
            
            
            if(temp_lx > 255.0f || temp_lx < 0.0f)
            {
                LX = last_lx;
            }
            else
            {
                LX = Kalman_Filter(&KF_LX,temp_lx); //temp_lx;// Kalman_Filter(&KF_LX,temp_lx);            // temp_lx*0.9 + last_lx*0.1 ;  
            }
            if(temp_ly>255.0f || temp_ly < 0.0f)
            {
                LY = last_ly;
            }
            else
            {
                LY =  Kalman_Filter(&KF_LY,temp_ly);// temp_ly;// Kalman_Filter(&KF_LY,temp_ly);                    //temp_ly * 0.9 + last_ly * 0.1; 
            }	
            
            last_lx = LX;
            last_ly = LY;
            
            //printf("%lf,%lf\r\n",LX,LY);
            
            debugPinTurn();
        }
        
		
		// �ڴ˴��������յ������ݣ�rxBuffer ��ǰ dataLength ���ֽڣ�
        
        //debugPinTurn();

        DMA_SetCurrDataCounter(DMA2_Channel3, BUFFER_SIZE_U4);  // �������� DMA ��������С
        
        DMA_Cmd(DMA2_Channel3, ENABLE);  // �������� DMA
    }
}


//void Serial_Init(void)
//{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//TX
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//RX
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	USART_InitTypeDef USART_InitStructure;
//	USART_InitStructure.USART_BaudRate =115200;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_Init(UART4, &USART_InitStructure);
//	
//	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
//	
//	
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//���ȼ�
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	USART_Cmd(UART4, ENABLE);
//	
//}



void Serial4_SendByte(uint8_t Byte)
{
	USART_SendData(UART4, Byte);
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial4_SendByte(Array[i]);
	}
}


void Serial_SendPacket(void)
{
	Serial4_SendByte(0xFF);
	Serial_SendArray(Serial_TxPacket, 4);
	Serial4_SendByte(0xFE);
}



uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

//void UART4_IRQHandler(void)
//{
//	static uint8_t RxState = 0;
//	static uint8_t pRxPacket = 0;
//	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
//	{
//		uint8_t RxData = USART_ReceiveData(UART4);
//		
//		if (RxState == 0)
//		{
//			if (RxData == 0xFF)
//			{
//				RxState = 1;
//				pRxPacket = 0;
//			}
//		}
//		else if (RxState == 1)
//		{
//			if (RxData != 0xFE)
//			{
//				Serial_RxPacket[pRxPacket] = RxData;
//				pRxPacket ++;
//			}
//			if (RxData == 0xFE)
//			{
//				RxState = 0;
//				Serial_RxFlag = 1;//�յ�һ���������ݵı�־λ
//				//mv�ش����ݣ����ݰ�ÿһλ�����ض����壬����һ���ض����������Ա����ͻ  
//				//instruction = [0XFF,0,0,0,0,0,0XFE]  5�����ñ���   �ڶ���Ϊ�߶��е�  
//				Serial_Yaw = Serial_RxPacket[0];//ֱ��ƫ�ƽǶ�
//				Last_Yaw = Serial_Yaw;
////				Serial_Yaw = Kalman_Filter(&KF_Angle,Serial_Yaw);
//				Last_Yaw = Kalman_Filter(&KF_Angle,Last_Yaw);
//				LX = Serial_RxPacket[2];			
//				LY = Serial_RxPacket[3];
//				Write_Flag = Serial_RxPacket[4];
//				Yellow_Flag = Serial_RxPacket[5];//ûɶ��
//			}
//		}

//		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
//	}
//}
//*******************************����4***************************************/



//*******************************����2***************************************/
#define BUFFER_SIZE_U2 256
uint8_t rxBuffer_U2[BUFFER_SIZE_U2];

void Serial_2_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // 1. ���� GPIOA��USART2 �� DMA1 ��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  // GPIOA ʱ��ʹ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // USART2 ʱ��ʹ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  // DMA1 ʱ��ʹ��

    // 2. ���� GPIOA2��TX���� GPIOA3��RX��Ϊ���ù���
    // ���� TX ���ţ�PA2��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ���� RX ���ţ�PA3��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // ��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. ���� USART2
    USART_DeInit(USART2);  // ��λ USART2 ����
    USART_InitStructure.USART_BaudRate = 115200;  // ���ò�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 1λֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;  // ��У��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // ���պͷ���ģʽ
    USART_Init(USART2, &USART_InitStructure);

    // ���� USART2 �Ŀ����ж�
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // �����ж�ʹ��

    // ���� NVIC �ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  // USART2 �ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // �����ж�
    NVIC_Init(&NVIC_InitStructure);

    // 4. ���� DMA ����
    DMA_DeInit(DMA1_Channel6);  // ��λ DMA ����
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;  // �����ַ��USART2 ���ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxBuffer_U2;  // �ڴ��ַ���������ݻ�����
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  // ���ݷ��򣺴����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE_U2;  // ��������С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // �����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // �������ݿ��ȣ��ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  // �ڴ����ݿ��ȣ��ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  // ����ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  // �����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  // �����ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);

    // ���� DMA
    DMA_Cmd(DMA1_Channel6, ENABLE);

    // ���� USART2 �� DMA ��������
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

    // 5. ���� USART2
    USART_Cmd(USART2, ENABLE);
}


//VOFA��λģʽPID����
void USART2_IRQHandler(void) 
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) 
	{
        USART_ReceiveData(USART2);  // �����ݼĴ�������� IDLE ��־
        DMA_Cmd(DMA1_Channel6, DISABLE);  // �ر� DMA

        uint16_t Length = BUFFER_SIZE_U2 - DMA_GetCurrDataCounter(DMA1_Channel6);  // ������ճ���
        
        // �ڴ˴��������յ������ݣ�rxBuffer_U2 ��ǰ dataLength ���ֽڣ�
        
        if(rxBuffer_U2[0]== 0x01) pidLX.kp += 0.001;
        if(rxBuffer_U2[0]== 0x02) pidLX.kp -= 0.001;
        
        if(rxBuffer_U2[0]== 0x03) pidLX.kd += 0.01;
        if(rxBuffer_U2[0]== 0x04) pidLX.kd -= 0.01;
        
        if(rxBuffer_U2[0]== 0x05) pidLX.kd += 0.001;
        if(rxBuffer_U2[0]== 0x06) pidLX.kd -= 0.001; 

		if(rxBuffer_U2[0] == 0xAF)
		{
			Write_Flag = 0xAF;	
		
		}

		
        if(rxBuffer_U2[0]== 0x11) pidLY.kp += 0.01;
        if(rxBuffer_U2[0]== 0x12) pidLY.kp -= 0.01;
        
        if(rxBuffer_U2[0]== 0x13) pidLY.kd += 0.01;
        if(rxBuffer_U2[0]== 0x14) pidLY.kd -= 0.01;
        
        if(rxBuffer_U2[0]== 0x15) pidLY.kd += 0.001;
        if(rxBuffer_U2[0]== 0x16) pidLY.kd -= 0.001; 
        
        if(rxBuffer_U2[0]== 0x21) pidRest(pPidObject,8);
        
        
        
        //Mode_Flag = REVOLVE_MODE_150 ; 
		 
        // debugPinToggle(); // ���Թ���
        
        
        
        // �ڴ˴��������յ������ݣ�rxBuffer_U2 ��ǰ dataLength ���ֽڣ�

        DMA_SetCurrDataCounter(DMA1_Channel6, BUFFER_SIZE_U2);  // �������� DMA ��������С
        
        DMA_Cmd(DMA1_Channel6, ENABLE);  // �������� DMA
    }
}

void Serial2_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial2_SendByte(Array[i]);
	}
}




int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);

    while (RESET == USART_GetFlagStatus(USART2, USART_FLAG_TXE))
    {
    }

    return ch;
}

#define PRINTF_BUFFER_SIZE 1024  // �趨�������Ĵ�С�����Ը�����Ҫ����

// ȫ�ֻ����������ڴ洢��ʽ������ַ���
char printf_buffer[PRINTF_BUFFER_SIZE];

void user_printf(const char *format, ...)
{
    va_list args;

    // ��ʼ���䳤�����б�
    va_start(args, format);

    // ��ʽ���ַ������洢��ȫ�ֻ����� printf_buffer ��
    vsnprintf(printf_buffer, sizeof(printf_buffer), format, args);

    // �����䳤�����ķ���
    va_end(args);

    // ����ֽڷ��͸�ʽ������ַ���
    Serial2_SendArray((uint8_t *)printf_buffer, strlen(printf_buffer));
}

//*******************************����2***************************************/


