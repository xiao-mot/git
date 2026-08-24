#ifndef __STEP_DIVER_H
#define __STEP_DIVER_H

#include "stm32f10x.h"
#include "Delay.h"

/**
 * @brief 初始化步进驱动器 ENA/DIR/PUL 引脚和 TIM7 更新中断。
 * @note ENA=PB4、DIR=PB5、PUL=PA12；当前驱动按共阴极接法编写。
 * @par 使用方法 `Micorstep_Driver_Init();`
 */
void Micorstep_Driver_Init(void);

/**
 * @brief 阻塞控制步进机构向上运动指定圈数。
 * @param number_of_turns 目标电机圈数，应为正数。
 * @note 调用前必须先执行 Micorstep_Enable()；函数会忙等到 TIM7 中断完成运动，不可在中断中调用。
 * @par 使用方法 `Micorstep_Enable(); UP(1.0f);`
 */
void UP(float number_of_turns);

/**
 * @brief 阻塞控制步进机构向下运动指定圈数。
 * @param number_of_turns 目标电机圈数，应为正数。
 * @note 调用前必须先执行 Micorstep_Enable()；无行程开关保护，必须确认机械限位。
 * @par 使用方法 `Micorstep_Enable(); DOWN(1.0f);`
 */
void DOWN(float number_of_turns);

/** @brief 软件允许下一次 UP()/DOWN() 运动。@note 每执行一次运动前都要调用；它不会直接控制硬件 ENA 电平。@par 使用方法 `Micorstep_Enable();` */
void Micorstep_Enable(void);

#endif
