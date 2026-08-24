#ifndef __IIC_H
#define __IIC_H

#include "stm32f10x.h"
#include "system_stm32f10x.h"

#define CLI() __set_PRIMASK(1)
#define SEI() __set_PRIMASK(0)

#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

#define TRUE 0
#define FALSE -1

#define IIC_RCC  RCC_APB2Periph_GPIOA
#define IIC_GPIO GPIOA
#define SCL_PIN  GPIO_Pin_5
#define SDA_PIN  GPIO_Pin_4

#define SCL_H    GPIOA->BSRR = GPIO_Pin_5
#define SCL_L    GPIOA->BRR  = GPIO_Pin_5
#define SDA_H    GPIOA->BSRR = GPIO_Pin_4
#define SDA_L    GPIOA->BRR  = GPIO_Pin_4
#define SCL_read GPIOA->IDR & GPIO_Pin_5
#define SDA_read GPIOA->IDR & GPIO_Pin_4

#define I2C_Direction_Trans 0
#define I2C_Direction_Rec   1

/** @brief 初始化软件 I2C 的 PA5(SCL) 和 PA4(SDA) 为开漏输出。@note 总线需要外部上拉电阻；该实现为阻塞式时序。@par 使用方法 `IIC_Init();` */
extern void IIC_Init(void);

/** @brief 向设备寄存器写 1 字节。@param addr 8 位写地址，例如 MPU6050 使用 0xD0。@param reg 寄存器地址。@param data 写入值。@return 0 成功，1 失败。@par 使用方法 `IIC_Write_One_Byte(0xD0, 0x6B, 0x00);` */
extern int8_t IIC_Write_One_Byte(uint8_t addr, uint8_t reg, uint8_t data);

/** @brief 从设备寄存器读取 1 字节。@param addr 8 位写地址，函数内部用 addr+1 读。@param reg 寄存器地址。@return 读取值；失败时也返回 1，无法与有效数据 0x01 区分。@par 使用方法 `id = IIC_Read_One_Byte(0xD0, 0x75);` */
extern int8_t IIC_Read_One_Byte(uint8_t addr, uint8_t reg);

/** @brief 连续写多个寄存器字节。@param addr 8 位写地址。@param reg 起始寄存器。@param data 数据缓冲区。@param len 字节数。@return 0 成功，1 失败。@par 使用方法 `IIC_Write_Bytes(0xD0, reg, data, len);` */
extern int8_t IIC_Write_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

/** @brief 连续读取多个寄存器字节。@param addr 8 位写地址。@param reg 起始寄存器。@param data 接收缓冲区。@param len 字节数。@return 0 成功，1 失败。@note data 必须至少有 len 字节空间。@par 使用方法 `IIC_read_Bytes(0xD0, 0x3B, data, 6);` */
extern int8_t IIC_read_Bytes(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

#endif
