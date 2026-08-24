#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"
#include "Delay.h"

/* TIM1 以 1 MHz 计数、20 ms 周期输出舵机 PWM，CCR 数值等于高电平微秒数。 */

// 转盘三个工位
#define stage_1 1850
#define stage_2 1150
#define stage_3 480

// 云台观察/收回位置
#define could_in  920
#define could_out 2150

// 机械爪释放/抓取位置
#define claw_free 1400
#define claw_grab 1820

// 步进升降机构位置参数（圈数）
#define start    2.1
#define platform 2.72
#define max      4.2

/** @brief 直接设置 TIM1 CH2 比较值，控制 PA9 机械爪舵机。@param Compare 高电平时间，单位 us，通常约 500~2500。@note 不限幅，必须先调用 Servo_Init()。@par 使用方法 `PWM1_SetCompare2(claw_free);` */
void PWM1_SetCompare2(uint16_t Compare);

/** @brief 直接设置 TIM1 CH3 比较值，控制 PA10 转盘舵机。@param Compare 高电平时间，单位 us。@note 不限幅。@par 使用方法 `PWM1_SetCompare3(stage_1);` */
void PWM1_SetCompare3(uint16_t Compare);

/** @brief 直接设置 TIM1 CH4 比较值，控制 PA11 云台舵机。@param Compare 高电平时间，单位 us。@note 不限幅。@par 使用方法 `PWM1_SetCompare4(could_out);` */
void PWM1_SetCompare4(uint16_t Compare);

/** @brief 以每次 10 us、间隔 20 ms 的斜坡把云台移动到目标位置。@param site 目标 CCR 值。@note 为阻塞函数，运动时间取决于当前位置与目标差值，不可在中断中调用。@par 使用方法 `cloud_tai(could_out);` */
void cloud_tai(int site);

/**
 * @brief 按 10 us 步进尝试移动机械爪。
 * @param site 期望的 CCR 位置。
 * @note 当前实现最后固定写入 500，并把内部位置重置为 claw_grab，行为与参数含义不一致；实车使用前必须单独验证。
 * @par 使用方法 `robotic_grab(claw_grab);`
 */
void robotic_grab(int site);

/** @brief 初始化 TIM1 CH2~CH4 的 50 Hz 舵机 PWM，并写入机械爪、转盘、云台初始位置。@note 上电时执行器会立即运动，调试前应断开负载或准备急停。@par 使用方法 `Servo_Init();` */
void Servo_Init(void);

#endif
