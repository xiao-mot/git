#ifndef __MOTOR_H
#define __MOTOR_H

#include "stdint.h"

/**
 * @brief 初始化四轮电机方向 GPIO，并调用 PWM_Init() 初始化 TIM8 四路 PWM。
 * @note 系统启动时调用一次。调试前应架空车轮；本函数没有主动写入停止 PWM。
 * @par 使用方法 `Motor_Init();`
 */
void Motor_Init(void);


/**
 * @brief 设置左前轮方向和 PWM，正负号决定方向，绝对值决定占空比。
 * @param lf_speed 建议范围 -999~999；0 表示 PWM 为 0。
 * @note 接口内部没有限幅，调用前必须先执行 Motor_Init()。
 * @par 使用方法 `Motor_left_front_wheel_SetSpeed(300);`
 */
void Motor_left_front_wheel_SetSpeed(int16_t lf_speed);

/**
 * @brief 设置右前轮方向和 PWM，正负号决定方向，绝对值决定占空比。
 * @param rfront_speed 建议范围 -999~999；0 表示 PWM 为 0。
 * @note 接口内部没有限幅，调用前必须先执行 Motor_Init()。
 * @par 使用方法 `Motor_right_front_wheel_SetSpeed(-300);`
 */
void Motor_right_front_wheel_SetSpeed(int16_t rfront_speed);

/**
 * @brief 设置左后轮方向和 PWM，正负号决定方向，绝对值决定占空比。
 * @param lb_speed 建议范围 -999~999；0 表示 PWM 为 0。
 * @note 接口内部没有限幅，调用前必须先执行 Motor_Init()。
 * @par 使用方法 `Motor_left_back_wheel_SetSpeed(300);`
 */
void Motor_left_back_wheel_SetSpeed(int16_t lb_speed);

/**
 * @brief 设置右后轮方向和 PWM，正负号决定方向，绝对值决定占空比。
 * @param rb_speed 建议范围 -999~999；0 表示 PWM 为 0。
 * @note 接口内部没有限幅，调用前必须先执行 Motor_Init()。
 * @par 使用方法 `Motor_right_back_wheel_SetSpeed(-300);`
 */
void Motor_right_back_wheel_SetSpeed(int16_t rb_speed);


#endif
