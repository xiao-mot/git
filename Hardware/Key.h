#ifndef __KEY_H
#define __KEY_H

/** @brief 初始化 PB6 为上拉输入按键。@par 使用方法 `Key_Init();` */
void Key_Init(void);

/** @brief 读取 PB6 按键并进行约 20 ms 阻塞消抖。@return 按下返回 1，松开返回 0。@note 按下和松开路径都会延时，不应在中断中调用。@par 使用方法 `if (Key_GetNum() == 1) { ... }` */
uint8_t Key_GetNum(void);

#endif
