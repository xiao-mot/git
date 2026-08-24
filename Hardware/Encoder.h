#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include "all_data.h"
#include "Filter.h"
#include "control.h"

/** @brief 初始化左前轮编码器 TIM3，输入为 PA6/PA7。@par 使用方法 `left_front_Encoder_Init();` */
void left_front_Encoder_Init(void);
/** @brief 初始化右前轮编码器 TIM2，部分重映射到 PA15/PB3。@note 会关闭 JTAG，保留 SWD。@par 使用方法 `right_front_Encoder_Init();` */
void right_front_Encoder_Init(void);
/** @brief 初始化左后轮编码器 TIM5，输入为 PA0/PA1。@par 使用方法 `left_back_Encoder_Init();` */
void left_back_Encoder_Init(void);
/** @brief 初始化右后轮编码器 TIM4，输入为 PB6/PB7。@par 使用方法 `right_back_Encoder_Init();` */
void right_back_Encoder_Init(void);

/**
 * @brief 一次性初始化四个轮子的编码器接口。
 * @note 通常由 ALL_Init() 调用；不要再调用 IC_Init()，因为它也使用 TIM3。
 * @par 使用方法 `Encoder_Init();`
 */
void Encoder_Init(void);

/**
 * @brief 读取四路增量计数并更新 F_L_Speed、F_R_Speed、B_L_Speed、B_R_Speed。
 * @note 每次读取都会清零四个定时器计数器，因此必须固定周期调用；当前 TIM6 中约每 5 ms 调用一次。
 * @par 使用方法 `Get_Speed();`
 */
void Get_Speed(void);

/** @brief 读取并清零左前轮 TIM3 计数。@return 上次读取后的有符号增量。@par 使用方法 `int16_t n = left_front_Encoder_Get();` */
int16_t left_front_Encoder_Get(void);
/** @brief 读取并清零右前轮 TIM2 计数。@return 上次读取后的有符号增量。@par 使用方法 `int16_t n = right_front_Encoder_Get();` */
int16_t right_front_Encoder_Get(void);
/** @brief 读取并清零左后轮 TIM5 计数。@return 上次读取后的有符号增量。@par 使用方法 `int16_t n = left_back_Encoder_Get();` */
int16_t left_back_Encoder_Get(void);
/** @brief 读取并清零右后轮 TIM4 计数，并在软件中反号。@return 修正方向后的增量。@par 使用方法 `int16_t n = right_back_Encoder_Get();` */
int16_t right_back_Encoder_Get(void);

/**
 * @brief 按目标轮圈数累计四轮里程，达到目标后把 Mode_Flag 置为 STOP_MODE。
 * @param circle 目标轮圈数，应为正数。
 * @note 当前实现的右后轮累计变量存在可疑写入，正式使用前应单轮测量验证。
 * @par 使用方法 `Get_wheel_circle(1.0f);`
 */
void Get_wheel_circle(float circle);

/**
 * @brief 将四路速度绝对值累计到里程计数变量 CNT_LF/CNT_RF/CNT_LB/CNT_RB。
 * @note 依赖 Get_Speed() 已经更新速度，累计值上限为 100000。
 * @par 使用方法 `Get_Speed(); Get_Dis();`
 */
void Get_Dis(void);//获取CNT计数

#endif
