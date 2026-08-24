#ifndef __IC_H
#define __IC_H

#include "stm32f10x.h"

/**
 * @brief 初始化 TIM3 CH1 输入捕获，PA6 上升沿到来时复位计数器并记录周期。
 * @note TIM3 同时被左前轮编码器使用，IC_Init() 与 Encoder_Init() 不能同时使用。
 * @par 使用方法 `IC_Init();`
 */
void IC_Init(void);

/**
 * @brief 根据 TIM3 捕获周期计算 72 MHz 时钟下的输入频率，并读取 PA4/PA5 判断方向。
 * @return uint32_t 形式的结果；反向分支写入负数后会发生无符号回绕。
 * @note 捕获值为 0 时会除以 0；PA4/PA5 也未在 IC_Init() 中初始化，当前接口不能直接用于车辆闭环。
 * @par 使用方法 `uint32_t value = IC_GetEncode();`
 */
uint32_t IC_GetEncode(void);

/** @brief 直接写指定定时器的 CCR1 寄存器。@param TIMx 定时器指针。@param Counter 新的 CCR1 值。@note 名称像设置计数器，实际写的是捕获比较寄存器 CCR1。@par 使用方法 `TIM_SetCCR(TIM3, 0);` */
void TIM_SetCCR(TIM_TypeDef* TIMx, uint16_t Counter);

#endif
