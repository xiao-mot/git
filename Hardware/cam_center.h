#ifndef _CAM_CENTER_H
#define _CAM_CENTER_H

#include <math.h>
#include <stdint.h>
#include "control.h"
#include <stdlib.h>

typedef struct
{
    float pixel_to_cm_x;  // X 方向像素到厘米的换算系数
    float pixel_to_cm_y;  // Y 方向像素到厘米的换算系数
    float CAM_CENTER_X;   // 图像中心 X 坐标
    float CAM_CENTER_Y;   // 图像中心 Y 坐标
    float MOVE_TOLERANCE; // 允许的像素误差
    uint8_t is_calibrated;// 标定完成标志
    uint8_t move_sequence;// 0 调整 X，1 调整 Y
} CamCenterCtrl;

extern CamCenterCtrl cam_ctrl;
extern float Location_distance ;

/**
 * @brief 设置像素换算系数默认值，并清除标定和移动序列状态。
 * @param ctrl 控制对象指针，不能为 NULL。
 * @note 本函数没有设置 CAM_CENTER_X、CAM_CENTER_Y 和 MOVE_TOLERANCE，调用方必须补充。
 * @par 使用方法 `CamCtrl_Init(&cam_ctrl);`
 */
void CamCtrl_Init(CamCenterCtrl* ctrl);

/**
 * @brief 根据已知实际移动距离与当前图像偏差计算 X/Y 像素换算系数。
 * @param ctrl 控制对象指针。
 * @param actual_move_cm 实际测量的移动距离，单位 cm。
 * @note 当中心坐标与 LX/LY 相等时会除以 0，标定前必须确保存在有效像素位移。
 * @par 使用方法 `CamCtrl_Calibrate(&cam_ctrl, 10.0f);`
 */
void CamCtrl_Calibrate(CamCenterCtrl* ctrl, float actual_move_cm);

/**
 * @brief 根据视觉坐标交替进行 X、Y 方向居中，修改 Mode_Flag 和 Location_distance。
 * @param ctrl 控制对象指针。
 * @param LX 当前目标 X 像素坐标。
 * @param LY 当前目标 Y 像素坐标。
 * @note 函数不直接驱动电机；应由固定周期状态机调用。当前实现对 float 使用 abs()，实车前需验证容差判断。
 * @par 使用方法 `CamCtrl_AutoCenter(&cam_ctrl, (int)LX, (int)LY);`
 */
void CamCtrl_AutoCenter(CamCenterCtrl* ctrl, int LX, int LY);

#endif
