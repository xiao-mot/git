#include "stm32f10x.h"                
#include <stdio.h>
#include <stdarg.h>
#include "hwt101uart.h"
#include "Delay.h"
#include "stdint.h"
#include "OLED.h"

//PB10 USART3-TX
//PB11 USART3-RX
//ȫ�ֱ���
volatile float global_angle = 0.0;
volatile float angular_velocity_y = 0.0;//���ٶ�y
volatile float angular_velocity_z = 0.0;//���ٶ�z
volatile uint8_t new_data_received = 0; //0��1



//������������
//�����Ĵ��� FF AA 69 88 B5
//Z��Ƕȹ���FF AA 76 00 00
//����200HZ��� FF AA 03 0B 00   //500hz
//���ò�����115200 FF AA 04 06 00
//���� FF AA 00 00 00
//���� FF AA 00 FF 00
uint8_t unlock_register[]     ={0xFF, 0xAA, 0x69, 0x88, 0xB5};
uint8_t reset_z_axis[]        ={0xFF, 0xAA, 0x76, 0x00, 0x00};
uint8_t set_output_200Hz[]    ={0xFF, 0xAA, 0x03, 0x0C, 0x00};
uint8_t set_baudrate_115200[] ={0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t save_settings[]       ={0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t restart_device[]      ={0xFF, 0xAA, 0x00, 0xFF, 0x00};
 

void Usart3_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate =115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ�������ƣ�����Ҫ
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                //ģʽ������ģʽ�ͽ���ģʽ��ѡ��
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //��żУ�飬����Ҫ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //ֹͣλ��ѡ��1λ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ���ѡ��8λ
	USART_Init(USART3, &USART_InitStructure);                                      //���ṹ���������USART_Init������Usart3
 
    /*USARTʹ��*/
    USART_Cmd(USART3, ENABLE); //ʹ��Usart3�����ڿ�ʼ����
	// ������������
    Usart3_SendArray(unlock_register, sizeof(unlock_register));
    Usart3_SendArray(reset_z_axis, sizeof(reset_z_axis));
    Usart3_SendArray(set_output_200Hz, sizeof(set_output_200Hz));
    Usart3_SendArray(set_baudrate_115200, sizeof(set_baudrate_115200));
    Usart3_SendArray(save_settings, sizeof(save_settings));
    Usart3_SendArray(restart_device, sizeof(restart_device));
	
} 



/*****************************************/
#define BUFFER_SIZE 256
uint8_t rxBuffer[BUFFER_SIZE];

void Usart3_Init_2(void)
{
    // ����ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // ���� GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ���� USART
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    // ���� USART3 �����ж�
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

    // ���� DMA
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel3); // USART3_RX ʹ�� DMA1 ͨ��3
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    // ���� DMA
    DMA_Cmd(DMA1_Channel3, ENABLE);

    // ���� USART3 �� DMA ���չ���
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

    // ���� NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ���� USART3
    USART_Cmd(USART3, ENABLE);

}



void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART3); // �����ݼĴ�������� IDLE ��־
        DMA_Cmd(DMA1_Channel3, DISABLE); // ��ʱ���� DMA

		// �������յ�������
        uint16_t receivedLength = BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3); // ��ȡ���յ����ݳ���

		if(receivedLength == 22 && rxBuffer[0] == 0x55 && rxBuffer[11] == 0x55)
		{
			uint8_t rxBUF_high[11];
			uint8_t rxBUF_low[11];
			
			for(int i = 0;i <= 10 ; i++)
			{
				rxBUF_high[i] = rxBuffer[i];
				rxBUF_low[i]  = rxBuffer[i+11];
			}
			
			ParseAndPrintData(rxBUF_high, 11);
			ParseAndPrintData(rxBUF_low, 11);
			
			//debugPinTurn();
		}
		// �������յ�������
		
		
        // �������� DMA ������
        DMA_SetCurrDataCounter(DMA1_Channel3, BUFFER_SIZE);
        DMA_Cmd(DMA1_Channel3, ENABLE); // �������� DMA
    }
}
/*****************************************/

 
void Usart3_SendByte(uint8_t Byte) {
    USART_SendData(USART3,Byte); //���ֽ�����д�����ݼĴ�����д���Usart3�Զ�����ʱ����
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); //�ȴ��������
}
 
void Usart3_SendString(char *String) {
    uint8_t i = 0;
    while (String[i]) { // �����ַ����飨�ַ������������ַ���������־λ��ֹͣ
        Usart3_SendByte(String[i]); // ���ε���Usart3_SendByte����ÿ���ֽ�����
        i++;
    }
}
 
void Usart3_Printf(char *format, ...) {
    char String[100];             //�����ַ�����
    va_list arg;                  //����ɱ�����б��������͵ı���arg
    va_start(arg, format);        //��format��ʼ�����ղ����б���arg����
    vsprintf(String, format, arg);//ʹ��vsprintf��ӡ��ʽ���ַ����Ͳ����б����ַ�������
    va_end(arg);                  //��������arg
    Usart3_SendString(String);    //���ڷ����ַ����飨�ַ�����
}
 
void Usart3_SendArray(uint8_t *array, uint16_t length) 
	{
    for (uint16_t i = 0; i < length; i++) {
        Usart3_SendByte(array[i]);
    }
    Usart3_SendByte(0x0D); // ���ͻس���
    Usart3_SendByte(0x0A); // ���ͻ��з�
	Delay_ms(200);
}
    
void Usart3_SendArray_2(uint8_t *array, uint16_t length) //����ʱ
{
    for (uint16_t i = 0; i < length; i++) {
        Usart3_SendByte(array[i]);
    }
    Usart3_SendByte(0x0D); // ���ͻس���
    Usart3_SendByte(0x0A); // ���ͻ��з�
}
 
void ParseAndPrintData(uint8_t *data, uint16_t length) 
{
    if (length == 11) 
    {
        uint8_t checksum = CalculateChecksum(data, length - 1, data[1]);
        if (checksum != data[length - 1]) {
            //Usart3_Printf("Checksum error\r\n");
            return;
        }
 
        if (data[0] == 0x55 && data[1] == 0x53) {
            // ���ݲ��ִ����� 6 ��ʼ�����ֽ���ǰ�����ֽ��ں�
            uint8_t yaw_l = data[6];
            uint8_t yaw_h = data[7];
            int16_t yaw = (int16_t)((yaw_h << 8) | yaw_l);
 
            // �������������ֵת��Ϊ�Ƕ�
            float angle = ((float)yaw / 32768.0) * 180.0;
            global_angle = angle;
            new_data_received = 1;
        } 
		else if (data[0] == 0x55 && data[1] == 0x52) {
            // �������ٶ�����
            uint8_t wy_l = data[4];
            uint8_t wy_h = data[5];
            int16_t wy = (int16_t)((wy_h << 8) | wy_l);
            angular_velocity_y = ((float)wy / 32768.0) * 2000.0;
 
            uint8_t wz_l = data[6];
            uint8_t wz_h = data[7];
            int16_t wz = (int16_t)((wz_h << 8) | wz_l);
            angular_velocity_z = ((float)wz / 32768.0) * 2000.0;
 
            new_data_received = 1;
        }
    }
}
 
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type) 
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}


//void USART3_IRQHandler(void) 
//{
//    static uint8_t rx_buffer[11];
//    static uint8_t rx_index = 0;//�����±�
//    static uint8_t state = 0;   //0: Ѱ�Ұ�ͷ��1: ��������
// 
//    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET )
//	{
//        uint8_t data = USART_ReceiveData(USART3);
//        if (state == 0)
//		{   // Ѱ�Ұ�ͷ
//            if (data == 0x55) 
//			{
//                rx_buffer[0] = data;
//                rx_index = 1;
//                state = 1;
//            }
//        } 
//		else if (state == 1) 
//		{   // ��������
//            rx_buffer[rx_index++] = data;
//            if (rx_index == 2 && rx_buffer[1] != 0x53 && rx_buffer[1] != 0x52)
//			{   // ����ڶ��ֽڲ���0x53��0x52�������Ѱ����һ��0x55
//                if (rx_buffer[1] == 0x55) { rx_index = 1; } else { state = 0; rx_index = 0; }
//            } 
//			else if (rx_index == 11) 
//			{   // ���յ������������ݰ�
//                ParseAndPrintData(rx_buffer, 11);
//                rx_index = 0;
//                state = 0; // ����Ѱ�Ұ�ͷ
//            }
//        }
// 
//        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//    }
//}

