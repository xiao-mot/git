#ifndef __LED_H
#define __LED_H

/** @brief 初始化 PA1/PA2 为推挽输出并置高，两个 LED 默认熄灭。@note LED 为低电平点亮。@par 使用方法 `LED_Init();` */
void LED_Init(void);
/** @brief 点亮 LED1（PA1 输出低）。@par 使用方法 `LED1_ON();` */
void LED1_ON(void);
/** @brief 熄灭 LED1（PA1 输出高）。@par 使用方法 `LED1_OFF();` */
void LED1_OFF(void);
/** @brief 翻转 LED1 当前状态。@par 使用方法 `LED1_Turn();` */
void LED1_Turn(void);
/** @brief 点亮 LED2（PA2 输出低）。@par 使用方法 `LED2_ON();` */
void LED2_ON(void);
/** @brief 熄灭 LED2（PA2 输出高）。@par 使用方法 `LED2_OFF();` */
void LED2_OFF(void);
/** @brief 翻转 LED2 当前状态。@par 使用方法 `LED2_Turn();` */
void LED2_Turn(void);

#endif
