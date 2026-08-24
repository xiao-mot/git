#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

/** @brief 使用 SysTick 产生阻塞微秒延时。@param us 延时时间，72 MHz 下单次最大约 233015 us。@note 硬编码 72 MHz，并会重写 SysTick 寄存器；不适合 RTOS 或中断上下文。@par 使用方法 `Delay_us(10);` */
void Delay_us(uint32_t us);

/** @brief 通过重复调用 Delay_us(1000) 产生阻塞毫秒延时。@param ms 延时时间。@note 延时期间 CPU 不执行主循环，不应放入控制中断。@par 使用方法 `Delay_ms(100);` */
void Delay_ms(uint32_t ms);

/** @brief 通过重复调用 Delay_ms(1000) 产生阻塞秒级延时。@param s 延时时间。@note 只适合简单上电流程或测试。@par 使用方法 `Delay_s(1);` */
void Delay_s(uint32_t s);

#endif
