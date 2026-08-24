#ifndef __TIMER4_H
#define __TIMER4_H

/** @brief 遗留声明，当前工程没有 Timer4_Init() 定义。@par 使用方法 当前不可调用，否则会链接失败。 */
void Timer4_Init(void);

/** @brief 遗留声明，当前工程没有 BaseTimer_Init() 定义。@par 使用方法 当前不可调用，否则会链接失败。 */
void BaseTimer_Init(void);

/**
 * @brief 初始化 TIM6 为 1 ms 更新中断，作为传感器、编码器、PID 和串口发送的调度时基。
 * @note 当前 ISR 约每 5 ms 更新姿态/编码器/PID，每 300 ms 发送一次数据；不要在主循环重复调用初始化。
 * @par 使用方法 `TIM6_Init();`
 */
void TIM6_Init(void);

#endif
