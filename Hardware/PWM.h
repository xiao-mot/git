#ifndef __PWM_H
#define __PWM_H

#include "stdint.h"

/**
 * @brief 初始化 TIM8 CH1~CH4 PWM，输出引脚为 PC6~PC9。
 * @note 当前 ARR=999，72 MHz 时钟下 PWM 约为 6 kHz，初始比较值为 0。
 * @par 使用方法 `PWM_Init();`
 */
void PWM_Init(void);

/** @brief 设置 TIM8 CH1（PC6）比较值。@param Compare 有效范围 0~999。@par 使用方法 `PWM_SetCompare1(300);` */
void PWM_SetCompare1(uint16_t Compare);
/** @brief 设置 TIM8 CH2（PC7）比较值。@param Compare 有效范围 0~999。@par 使用方法 `PWM_SetCompare2(300);` */
void PWM_SetCompare2(uint16_t Compare);
/** @brief 设置 TIM8 CH3（PC8）比较值。@param Compare 有效范围 0~999。@par 使用方法 `PWM_SetCompare3(300);` */
void PWM_SetCompare3(uint16_t Compare);
/** @brief 设置 TIM8 CH4（PC9）比较值。@param Compare 有效范围 0~999。@par 使用方法 `PWM_SetCompare4(300);` */
void PWM_SetCompare4(uint16_t Compare);

#endif
