#ifndef __HWT101UART_H
#define __HWT101UART_H

#include <stdio.h>
#include "stdint.h"

/* HWT101 配置命令，每个数组均为 5 字节。 */
extern uint8_t unlock_register[5];
extern uint8_t reset_z_axis[5];
extern uint8_t set_output_200Hz[5];
extern uint8_t set_baudrate_115200[5];
extern uint8_t save_settings[5];
extern uint8_t restart_device[5];

/**
 * @brief 初始化 USART3（PB10/PB11，115200-8-N-1），并依次向 HWT101 发送配置命令。
 * @note 此版本不启用接收 DMA；每条命令后延时约 200 ms，整体为阻塞初始化。
 * @par 使用方法 `Usart3_Init();`
 */
void Usart3_Init(void);

/** @brief 通过 USART3 阻塞发送 1 字节。@param Byte 待发送字节。@note 必须先初始化 USART3。@par 使用方法 `Usart3_SendByte(0x55);` */
void Usart3_SendByte(uint8_t Byte);

/** @brief 通过 USART3 阻塞发送字符串。@param String 以 '\0' 结束的字符串。@par 使用方法 `Usart3_SendString("AT\r\n");` */
void Usart3_SendString(char *String);

/** @brief 发送数组后追加 CR/LF，并额外阻塞 200 ms。@param array 数据首地址。@param length 字节数。@par 使用方法 `Usart3_SendArray(cmd, sizeof(cmd));` */
void Usart3_SendArray(uint8_t *array, uint16_t length);

/** @brief 发送数组后追加 CR/LF，但不增加 200 ms 延时。@param array 数据首地址。@param length 字节数。@par 使用方法 `Usart3_SendArray_2(cmd, sizeof(cmd));` */
void Usart3_SendArray_2(uint8_t *array, uint16_t length);

/** @brief 格式化后通过 USART3 阻塞发送。@param format printf 风格格式串。@note 仅有 100 字节栈缓冲且使用 vsprintf，过长文本会越界。@par 使用方法 `Usart3_Printf("yaw=%.2f\r\n", global_angle);` */
void Usart3_Printf(char *format, ...);

/** @brief 校验并解析一帧 11 字节 HWT101 数据。@param data 帧首地址。@param length 必须为 11。@note 0x53 帧更新 global_angle，0x52 帧更新 Y/Z 角速度。@par 使用方法 `ParseAndPrintData(frame, 11);` */
void ParseAndPrintData(uint8_t *data, uint16_t length);

/** @brief 对前 length 字节执行 8 位累加校验。@param data 数据首地址。@param length 参与校验的字节数。@param type 当前实现未使用此参数。@return 累加和低 8 位。@par 使用方法 `sum = CalculateChecksum(frame, 10, frame[1]);` */
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type);

/**
 * @brief 初始化 USART3 和 DMA1_Channel3 循环接收，并启用空闲中断解析 HWT101 数据。
 * @note 此版本不会发送传感器配置命令；Usart3_Init() 与 Usart3_Init_2() 应按需求选择，不要重复初始化。
 * @par 使用方法 `Usart3_Init_2();`
 */
void Usart3_Init_2(void);

#endif
