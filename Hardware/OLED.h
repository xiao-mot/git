#ifndef __OLED_H
#define __OLED_H

#include "stdint.h"

/* 调试引脚和 OLED 软件 I2C 共用 PC4/PC5，使用时必须注意引脚冲突。 */

/** @brief 初始化 PC4/PC5 为推挽输出并置高。@note PC4/PC5 同时是 OLED SCL/SDA，初始化后会改变 OLED 总线状态。@par 使用方法 `debugPinInit();` */
void debugPinInit(void);
/** @brief 将调试引脚 PC4 置高。@note 会直接改变 OLED SCL。@par 使用方法 `debugPinON();` */
void debugPinON(void);
/** @brief 将调试引脚 PC4 置低。@note 会直接改变 OLED SCL。@par 使用方法 `debugPinOFF();` */
void debugPinOFF(void);
/** @brief 翻转调试引脚 PC4，便于示波器测量代码执行时刻。@note 会直接改变 OLED SCL。@par 使用方法 `debugPinTurn();` */
void debugPinTurn(void);

/** @brief 输出两次短蜂鸣提示。@note 内部阻塞约 170 ms，并通过 PC4 输出。@par 使用方法 `Buzzer_bb();` */
void Buzzer_bb(void);
/** @brief 输出一次短蜂鸣提示。@note 内部阻塞约 70 ms，并通过 PC4 输出。@par 使用方法 `Buzzer_b();` */
void Buzzer_b(void);
/** @brief 使用 PC4 软件翻转播放内置旋律。@note 长时间阻塞，会影响控制周期，不可在中断中调用。@par 使用方法 `play_music();` */
void play_music(void);

/** @brief 初始化 128x64 OLED 的 PC4/PC5 软件 I2C，并发送 SSD1306 配置和清屏。@note 上电后调用一次。@par 使用方法 `OLED_Init();` */
void OLED_Init(void);
/** @brief 清空 OLED 全部 8 页显示 RAM。@note 为阻塞式软件 I2C 操作。@par 使用方法 `OLED_Clear();` */
void OLED_Clear(void);

/** @brief 显示一个 8x16 ASCII 字符。@param Line 行号 1~4。@param Column 列号 1~16。@param Char 可见 ASCII 字符。@par 使用方法 `OLED_ShowChar(1, 1, 'A');` */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
/** @brief 显示 8x16 ASCII 字符串。@param Line 起始行 1~4。@param Column 起始列 1~16。@param String 以 '\0' 结束的字符串。@note 调用方需保证字符串不越过屏幕。@par 使用方法 `OLED_ShowString(1, 1, "READY");` */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
/** @brief 以固定宽度显示无符号十进制数。@param Line 行 1~4。@param Column 列 1~16。@param Number 数值。@param Length 显示位数 1~10。@par 使用方法 `OLED_ShowNum(2, 1, 123, 3);` */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
/** @brief 显示带正负号的十进制整数。@param Line 行 1~4。@param Column 列 1~16。@param Number 数值。@param Length 不含符号的数字位数。@par 使用方法 `OLED_ShowSignedNum(2, 1, -12, 2);` */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
/** @brief 以固定宽度显示十六进制数。@param Line 行 1~4。@param Column 列 1~16。@param Number 数值。@param Length 位数 1~8。@par 使用方法 `OLED_ShowHexNum(3, 1, 0x5A, 2);` */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
/** @brief 以固定宽度显示二进制数。@param Line 行 1~4。@param Column 列 1~16。@param Number 数值。@param Length 位数 1~32。@par 使用方法 `OLED_ShowBinNum(4, 1, 5, 4);` */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/** @brief 显示一个 16x32 大号 ASCII 字符。@param Line 行号 1~2。@param Column 列号 1~8。@param Char 可见 ASCII 字符。@par 使用方法 `OLED_ShowChar_high(1, 1, 'A');` */
void OLED_ShowChar_high(uint8_t Line, uint8_t Column, char Char);
/** @brief 显示 16x32 大号字符串。@param Line 起始行 1~2。@param Column 起始列 1~8。@param String 字符串。@note 调用方需保证不越过屏幕。@par 使用方法 `OLED_ShowString_high(1, 1, "12");` */
void OLED_ShowString_high(uint8_t Line, uint8_t Column, char *String);
/** @brief 以大号字体显示二进制数。@param Line 行 1~2。@param Column 列 1~8。@param Number 数值。@param Length 位数。@par 使用方法 `OLED_ShowBinNum_high(1, 1, 3, 2);` */
void OLED_ShowBinNum_high(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
/** @brief 以大号字体显示十六进制数。@param Line 行 1~2。@param Column 列 1~8。@param Number 数值。@param Length 位数。@par 使用方法 `OLED_ShowHexNum_high(1, 1, 0x2A, 2);` */
void OLED_ShowHexNum_high(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
/** @brief 以大号字体显示带符号整数。@param Line 行 1~2。@param Column 列 1~8。@param Number 数值。@param Length 不含符号的位数。@par 使用方法 `OLED_ShowSignedNum_high(1, 1, -8, 1);` */
void OLED_ShowSignedNum_high(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
/** @brief 以大号字体显示无符号十进制数。@param Line 行 1~2。@param Column 列 1~8。@param Number 数值。@param Length 位数。@par 使用方法 `OLED_ShowNum_high(1, 1, 25, 2);` */
void OLED_ShowNum_high(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

/** @brief 以普通字体显示保留两位小数的浮点数。@param Line 行 1~4。@param Column 列 1~16。@param Number 浮点值。@note 内部使用 sprintf，耗时和代码体积较大。@par 使用方法 `OLED_ShowDecimals(3, 2, 12.34f);` */
void OLED_ShowDecimals(uint8_t Line, uint8_t Column, float Number);
/** @brief 以大号字体显示保留两位小数的浮点数。@param Line 行 1~2。@param Column 列 1~8。@param Number 浮点值。@note 字符较多时容易超出屏幕。@par 使用方法 `OLED_ShowDecimals_high(1, 1, 3.14f);` */
void OLED_ShowDecimals_high(uint8_t Line, uint8_t Column, float Number);

#endif
