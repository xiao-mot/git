#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"
#include "ALL_DATA.h"

/**
 * @brief 执行串级 PID：先计算外环 pidAngE，再把外环输出作为内环 pidRate 的目标值。
 * @param pidRate 内环 PID 对象指针。
 * @param pidAngE 外环 PID 对象指针。
 * @param dt 两次调用间隔，单位应为秒且必须大于 0。
 * @note 函数不检查空指针，也不执行输出限幅。
 * @par 使用方法 `CascadePID(&pidRateZ, &pidYaw, 0.005f);`
 */
extern void CascadePID(PidObject* pidRate, PidObject* pidAngE, const float dt);

/**
 * @brief 清零一组 PID 的积分、上次误差、输出和偏移量。
 * @param pid PID 指针数组，例如 pPidObject。
 * @param len 数组中有效指针的数量。
 * @note 不检查数组越界和空指针，len 必须与实际数组长度一致。
 * @par 使用方法 `pidRest(pPidObject, 8);`
 */
extern void pidRest(PidObject **pid, const uint8_t len);

/**
 * @brief 根据 desired、measured 和 dt 更新单个 PID 的积分、微分和输出。
 * @param pid 待更新的 PID 对象。
 * @param dt 采样周期，单位应为秒且必须大于 0。
 * @note 当前实现中的积分限幅和输出限幅代码被注释，调用方必须自行限幅。
 * @par 使用方法 `pidUpdate(&PID_FOR_L, 0.005f);`
 */
extern void pidUpdate(PidObject* pid, const float dt);

#endif
