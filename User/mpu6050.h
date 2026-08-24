#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "stm32f10x.h"
#include "delay.h"
#include "IIC.h"
#include "flash.h"
#include <string.h>
#include <math.h>
#include "all_data.h"

#define squa( Sq )        (((float)Sq)*((float)Sq))
/** 一维卡尔曼滤波状态，仅由 mpu6050.c 内部辅助函数使用。 */
struct _1_ekf_filter
{
    float LastP;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;
};

/** 四元数姿态状态。 */
typedef volatile struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

/** 三轴浮点向量。 */
struct V
{
    float x;
    float y;
    float z;
};

/**
 * @brief 通过软件 I2C 复位并配置 MPU6050，最后读取 WHO_AM_I 校验设备。
 * @return 0 表示成功，1 表示失败。
 * @note 调用前必须先执行 IIC_Init()；配置写入失败时当前 do-while 可能无限阻塞。
 * @par 使用方法 `IIC_Init(); if (MpuInit() != 0) { ... }`
 */
int8_t MpuInit(void);

/**
 * @brief 连续读取加速度和角速度共 12 字节，扣除偏置并进行简单滤波。
 * @note 结果写入全局 MPU6050；函数不返回 I2C 错误状态，建议按固定采样周期调用。
 * @par 使用方法 `MpuGetData();`
 */
void MpuGetData(void);

/**
 * @brief 静止采样并计算六轴零偏。
 * @note 会暂时关闭 TIM6 中断并执行数百次阻塞采样；校准期间传感器和小车必须保持静止。
 * @par 使用方法 `MpuGetOffset();`
 */
void MpuGetOffset(void);

/**
 * @brief 使用加速度计和陀螺仪更新四元数，并计算 roll、pitch、yaw。
 * @param pMpu 六轴原始数据指针，不能为 NULL。
 * @param pAngE 姿态输出指针，不能为 NULL。
 * @param dt 实际采样周期，单位秒且必须大于 0。
 * @note 调用前应先执行 MpuGetData()；dt 必须与真实调度周期一致。
 * @par 使用方法 `MpuGetData(); GetAngle(&MPU6050, &Angle, 0.005f);`
 */
void GetAngle(const _st_Mpu *pMpu, _st_AngE *pAngE, float dt);

extern _st_Mpu MPU6050;
extern _st_AngE Angle;

#endif
