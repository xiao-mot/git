#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>

/* UART4 发送缓冲区、UART4 旧接收缓冲区及旧接收完成标志。 */
extern uint8_t Serial_TxPacket[10];
extern uint8_t Serial_RxPacket[10];
extern uint8_t Serial_RxFlag;

/* ---------------- UART5：二维码/任务码通信 ---------------- */

/** @brief 通过 UART5 阻塞发送 1 字节。@param Byte 待发送字节。@note 必须先调用 Serial_5_Init()，不建议在中断中调用。@par 使用方法 `Serial_SendByte(0x55);` */
void Serial_SendByte(uint8_t Byte);

/** @brief 通过 UART4 阻塞发送字节数组。@param Array 数据首地址，不能为 NULL。@param Length 字节数。@note 名称没有体现 UART4，容易与 UART5 接口混淆。@par 使用方法 `Serial_SendArray(data, sizeof(data));` */
void Serial_SendArray(uint8_t *Array, uint16_t Length);

/** @brief 通过 UART5 阻塞发送以 '\0' 结束的字符串。@param String 字符串指针。@par 使用方法 `Serial_SendString("OK\r\n");` */
void Serial_SendString(char *String);

/** @brief 通过 UART5 发送指定宽度的无符号十进制数。@param Number 数值。@param Length 固定显示位数。@note 超出位数的高位会被截掉。@par 使用方法 `Serial_SendNumber(123, 3);` */
void Serial_SendNumber(uint32_t Number, uint8_t Length);

/** @brief 格式化后通过 UART5 阻塞发送。@param format printf 风格格式串。@note 内部仅有 100 字节栈缓冲且使用 vsprintf，过长文本会造成越界；不要在中断中调用。@par 使用方法 `Serial_Printf("x=%d\r\n", x);` */
void Serial_Printf(char *format, ...);

/** @brief 初始化 UART4（PC10/PC11，115200-8-N-1）和 DMA2_Channel3 循环接收。@note 空闲中断会直接解析视觉数据并更新 LX、LY、Write_Flag 等全局量。@par 使用方法 `Serial_4_Init();` */
void Serial_4_Init(void);

/** @brief 初始化 UART5（PC12/PD2，115200-8-N-1）和逐字节接收中断。@note 接收协议以 0xFF 开头、0xFE 结尾，当前实现没有接收数组边界保护。@par 使用方法 `Serial_5_Init();` */
void Serial_5_Init(void);

/** @brief 通过 UART4 发送一帧：0xFF + Serial_TxPacket 前 4 字节 + 0xFE。@note 为阻塞发送，必须先初始化 UART4。@par 使用方法 `Serial_SendPacket();` */
void Serial_SendPacket(void);

/** @brief 读取并清除旧版 UART4 接收完成标志。@return 有完整旧协议包时返回 1，否则返回 0。@note 当前启用的 UART4 DMA 中断没有置位 Serial_RxFlag，因此此接口通常返回 0。@par 使用方法 `if (Serial_GetRxFlag()) { ... }` */
uint8_t Serial_GetRxFlag(void);

/** @brief 通过 UART5 阻塞发送数组。@param Array 数据首地址。@param Length 字节数。@par 使用方法 `Serial5_SendArray(data, len);` */
void Serial5_SendArray(uint8_t *Array, uint16_t Length);

/* ---------------- USART2：调参/调试输出 ---------------- */

/** @brief 初始化 USART2（PA2/PA3，115200-8-N-1）和 DMA1_Channel6 循环接收。@note 接收中断按首字节直接修改定位 PID 参数。@par 使用方法 `Serial_2_Init();` */
void Serial_2_Init(void);

/** @brief 通过 USART2 阻塞发送 1 字节。@param Byte 待发送字节。@par 使用方法 `Serial2_SendByte(0x21);` */
void Serial2_SendByte(uint8_t Byte);

/** @brief 通过 USART2 阻塞发送数组。@param Array 数据首地址。@param Length 字节数。@par 使用方法 `Serial2_SendArray(data, len);` */
void Serial2_SendArray(uint8_t *Array, uint16_t Length);

/** @brief 格式化文本后通过 USART2 发送。@param format printf 风格格式串。@note 使用全局 1024 字节缓冲区，不可重入，不应在中断中调用。@par 使用方法 `user_printf("speed=%.2f\r\n", F_L_Speed);` */
void user_printf(const char *format, ...);

/** @brief 遗留占位声明，当前工程没有函数定义。@par 使用方法 当前不可调用，否则会链接失败。 */
void c(void);

#endif
