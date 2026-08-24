#include "iic.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

void I2c_delay()  {
    volatile unsigned char i = 2;
    while (i)
        i--;
}

/******************************************************************************
 * ��������: I2c_Init
 * ��������: I2c  GPIO��ʼ��
 * ��ڲ���: ��
 ******************************************************************************/
 void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStrucSUCCESS;
    RCC_APB2PeriphClockCmd(IIC_RCC  , ENABLE);
	    
    GPIO_InitStrucSUCCESS.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStrucSUCCESS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStrucSUCCESS.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(IIC_GPIO, &GPIO_InitStrucSUCCESS);
}

/******************************************************************************
 * ��������: I2c_Start
 * ��������: I2c  ��ʼ�ź�
 * ��ڲ���: ��
 ******************************************************************************/
static uint8_t I2c_Start(void)
{
    SDA_H;
    SCL_H;
	I2c_delay();
    if (!SDA_read)
        return FAILED;
    SDA_L;
    I2c_delay();
    if (SDA_read)
        return FAILED;
    SCL_L;
    I2c_delay();
    return SUCCESS;
}

/******************************************************************************
 * ��������: I2c_Stop
 * ��������: I2c  ֹͣ�ź�
 * ��ڲ���: ��
 ******************************************************************************/
static void I2c_Stop(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
	I2c_delay();
    I2c_delay();
    SCL_H;
	I2c_delay();
    SDA_H;
    I2c_delay();
}


static void I2c_Ack(void)
{
    SCL_L;
    I2c_delay();
    SDA_L;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}


static void I2c_NoAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
	I2c_delay();
    I2c_delay();
    SCL_L;
    I2c_delay();
}

/*******************************************************************************
 *��������:	I2c_WaitAck
 *��������:	�ȴ�Ӧ���źŵ���
 *����ֵ��   1������Ӧ��ʧ��
 *           0������Ӧ��ɹ�
 *******************************************************************************/
static uint8_t I2c_WaitAck(void)
{
    SCL_L;
    I2c_delay();
    SDA_H;
    I2c_delay();
    SCL_H;
	I2c_delay();
	I2c_delay();
    I2c_delay();
	
    if (SDA_read) {
        SCL_L;
        return FAILED;
    }
    SCL_L;
    return SUCCESS;
}

/******************************************************************************
 * ��������: I2c_SendByte
 * ��������: I2c  ����һ���ֽ�����
 * ��ڲ���: byte  ���͵�����
 ******************************************************************************/
static void I2c_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        I2c_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2c_delay();
        SCL_H;
		I2c_delay();
		I2c_delay();
		I2c_delay();
    }
    SCL_L;
}

/******************************************************************************
 * ��������: I2c_ReadByte
 * ��������: I2c  ��ȡһ���ֽ�����
 * ��ڲ���: ��
 * ����ֵ	 ��ȡ������
 ******************************************************************************/
static uint8_t I2c_ReadByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2c_delay();
		I2c_delay();
        SCL_H;
		I2c_delay();
        I2c_delay();
		I2c_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

/******************************************************************************
 * ��������: i2cWriteBuffer
 * ��������: I2c       ���豸��ĳһ����ַд��̶����ȵ�����
 * ��ڲ���: addr,     �豸��ַ
 *           reg��     �Ĵ�����ַ
 *			 len��     ���ݳ���
 *			 *data	   ����ָ��
 * ����ֵ	 1
 ******************************************************************************/
int8_t IIC_Write_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{
    int i;
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    for (i = 0; i < len; i++) {
        I2c_SendByte(data[i]);
        if (I2c_WaitAck() == FAILED) {
            I2c_Stop();
            return FAILED;
        }
    }
    I2c_Stop();
    return SUCCESS;
}
int8_t IIC_Read_One_Byte(uint8_t addr,uint8_t reg)
{
	uint8_t recive = 0;
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
	I2c_Stop();
    I2c_Start();
	I2c_SendByte(addr+1);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
	recive = I2c_ReadByte();
	 I2c_NoAck();
	I2c_Stop();
	return recive;
}

/*****************************************************************************
 *��������:	i2cWrite
 *��������:	д��ָ���豸 ָ���Ĵ���һ���ֽ�
 *��ڲ����� addr Ŀ���豸��ַ
 *		     reg   �Ĵ�����ַ
 *		     data ���������ݽ�Ҫ��ŵĵ�ַ
 *******************************************************************************/
int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
    I2c_SendByte(data);
    I2c_WaitAck();
    I2c_Stop();
    return SUCCESS;
}

int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len)
{
    if (I2c_Start() == FAILED)
        return FAILED;
    I2c_SendByte(addr);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    I2c_SendByte(reg);
    I2c_WaitAck();
	I2c_Stop();
    I2c_Start();
    I2c_SendByte(addr+1);
    if (I2c_WaitAck() == FAILED) {
        I2c_Stop();
        return FAILED;
    }
    while (len) {
        *data = I2c_ReadByte();
        if (len == 1)
            I2c_NoAck();
        else
            I2c_Ack();
        data++;
        len--;
    }
    I2c_Stop();
    return SUCCESS;
}





