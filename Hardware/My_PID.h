#ifndef __MY_PID_H
#define __MY_PID_H

typedef struct
{
    float desire;          // 目标值
    float measured_value;  // 测量值
    float integral;        // 误差积分
    float derivative;      // 误差微分
    float kp;              // 比例系数
    float ki;              // 积分系数
    float kd;              // 微分系数
    float IntegLimitHigh;  // 积分上限
    float IntegLimitLow;   // 积分下限
    float OutLimitHigh;    // 输出上限
    float OutLimitLow;     // 输出下限
    float error0;          // 当前误差
    float error1;          // 上次误差
    float out;             // PID 输出
} PID_Structure;

/** @brief 给全局 PidYaw 写入默认 Kp/Ki/Kd。
 * @note 不初始化状态和限幅。
 * @par 使用方法 `PID_ParamInit();`
 *  */
void PID_ParamInit(void);

/** @brief 清零指定 PID 对象的目标、测量、误差、积分、微分和输出。
 * @param pid 不能为 NULL。
 * @par 使用方法 `PID_Date_Init(&PidYaw);` 
 * */
void PID_Date_Init(PID_Structure* pid);

/** @brief 给全局 PidYaw 设置输出上下限。
 * @note 当前函数没有设置积分上下限。
 * @par 使用方法 `PID_Limt_Init();` 
 * */
void PID_Limt_Init(void);
/** @brief 更新全局 PidYaw 的目标值和测量值。
 * @note 当前实现把两者都强制写成 0，尚未接入真实传感器。
 * @par 使用方法 通常由 PID_Control() 内部调用。 
 * */
void pid_update(void);

/**
 * @brief 按固定 0.05 s 周期计算 PID，并把结果同步到全局 Yaw_Out。
 * @param pid PID 对象指针，不能为 NULL。
 * @note 当前会先调用 pid_update()，因此目标和测量都会被重置为 0；此模块不是现有车辆主控制 PID。
 * @par 使用方法 `PID_Control(&PidYaw);`
 */
void PID_Control(PID_Structure* pid);

#endif
