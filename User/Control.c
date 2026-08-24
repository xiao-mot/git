/**
 * @file    control.c
 * @brief   物流搬运小车运动控制与机械臂动作管理模块
 * @note    包含多种运动模式（前进、后退、旋转、平移、位置闭环等）、
 *          PID 级联控制、视觉定位、机械爪/转台/升降台动作序列、以及
 *          完整任务状态机 (LineMove_Mode)。
 *          依赖：pid.h 提供 PID 更新和复位函数。
 */

#include "control.h"
#include <stdint.h>
#include "hwt101uart.h"
#include <stdio.h>
#include <stdarg.h>
#include "stdint.h"
#include "OLED.h"

/* 四轮编码器一圈脉冲数（Encoder.c 定义），横移时用于按轮归一化物理转速 */
extern const float CNT_LF_turn, CNT_LB_turn, CNT_RF_turn, CNT_RB_turn;

/*==================== 全局运行模式与状态变量 =====================*/
unsigned char Mode_Flag = STOP_MODE;   // 当前运动模式，由主循环调度

unsigned char Last_Mode_Flag = 0xFF;   // 上一次调度模式，用于检测模式切换瞬间

uint8_t Location_effective = 0;        // 定位有效计数（达到阈值后才认为定位完成）
unsigned int  Timer = 0;               // 系统定时器计数（用于超时等）
unsigned char NowStep = 0;             // 当前任务步骤（1~7，用于状态机）
unsigned char Catch_Flag = 0;          // 抓取动作进行标志





//=======================================视觉+抓取=======================================================
/* 颜色搜索标志（用于视觉识别） */
unsigned char Find_Yellow2 = 0;
unsigned char Search_Flag =  0;
unsigned char Search_Flag2 = 0;

/* 放置/抓取阶段标志（区分不同颜色物料） */
unsigned char Find1 = 1,   Find2,   Find3;      // 第1/2/3种物料是否在寻找放置位置
unsigned char Catch1 = 1,  Catch2,  Catch3;     // 第1/2/3种物料是否在寻找抓取
unsigned char Find2_1 = 1, Find2_2, Find2_3;
unsigned char Catch2_1 = 1,Catch2_2,Catch2_3;

unsigned char Catch_object;                     // 当前正在抓取的对象编号

unsigned char P_1, P_2, P_3;                    // 放置阶段各颜色标志（是否已发送视觉指令）
unsigned char C_1, C_2, C_3;                    // 抓取阶段各颜色标志

unsigned char Place_Over_Red = 0;               // 红色放置完成
unsigned char Place_Over_Blue = 0;              // 蓝色放置完成
unsigned char Place_Over_Green = 0;             // 绿色放置完成

unsigned char Catch_Over_Red = 0;               // 红色抓取完成
unsigned char Catch_Over_Blue = 0;
unsigned char Catch_Over_Green = 0;

unsigned char Place_Red = 0;                    // 供主程序使用的放置完成标志
unsigned char Place_Blue = 0;
unsigned char Place_Green = 0;

unsigned char Catch_Red = 0;                    // 供主程序使用的抓取完成标志
unsigned char Catch_Green = 0;
unsigned char Catch_Blue = 0;

/*==================== PID 对象指针数组 =====================*/
PidObject *(pPidObject[]) = {
    &PID_FOR_L,   // 左前轮速度环
    &PID_FOR_R,   // 右前轮速度环
    &PID_BAC_L,   // 左后轮速度环
    &PID_BAC_R,   // 右后轮速度环
    &pidYaw,      // 偏航角环（外环）
    &pidRateZ,    // 偏航角速度环（内环）
    &pidLX,       // X方向位置环（视觉定位）
    &pidLY        // Y方向位置环（视觉定位）
};

/**
 * @brief   根据当前 Mode_Flag 调用对应的运动模式函数
 * @note    此函数应在定时器中断或主循环中周期性调用，
 *          所有模式函数内部均会更新四个轮子的 PID 输出，
 *          最后统一调用 Control_Moto() 输出 PWM。
 */
void Pid_Control_Trans()
{
    if (Mode_Flag == REVOLVE_MODE) {
        Revolve_Mode(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == STOP_MODE) {
        Stop_Mode();
    }
    else if (Mode_Flag == FOR_MODE) {
        Forward_Mode(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == TRANS_MODE) {
        Trans_Mode(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == LOCATION_MODE) {
        Location_Mode(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == TRANS_LEFT_MODE) {
        Trans_Left_Mode_Run(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == BACK_MODE) {
        Back_Mode(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == LOCATION_PLACE_MODE) {
        Location_Mode_Place(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == FOR_MODE_LOW) {
        Forward_Mode_Low(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == REVOLVE_MODE_150) {
        Revolve_Mode_150(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == REVOLVE_MODE_0) {
        Revolve_Mode_0(0.005f);
        Control_Moto();
    }
    else if (Mode_Flag == BACK_MODE_Low) {
        Back_Mode_Low(0.005f);
        Control_Moto();
    }

    Last_Mode_Flag = Mode_Flag;
}

/*==================== 各种运动模式函数 =====================*/

/**
 * @brief   原地旋转到目标偏航角（目标角=0°）
 * @note    仅偏航角单环控制，不涉及角速度内环。
 */
void Revolve_Mode_0(float dt)
{
    // 偏航角 PID：目标为 Angle_Yaw（外部设定），测量为 Angle.yaw
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    // 四轮速度设定：左右轮反向，实现原地旋转
    PID_FOR_L.desired = -pidYaw.out;
    PID_FOR_R.desired = +pidYaw.out;
    PID_BAC_L.desired = -pidYaw.out;
    PID_BAC_R.desired = +pidYaw.out;

    // 实际轮速反馈
    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    // 更新各轮速度 PID
    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   原地旋转到目标偏航角（目标角=150°）
 */
void Revolve_Mode_150(float dt)
{
    pidYaw.desired = Angle_Yaw;   // 目标角预设为150°（外部赋值）
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    PID_FOR_L.desired = -pidYaw.out;
    PID_FOR_R.desired = +pidYaw.out;
    PID_BAC_L.desired = -pidYaw.out;
    PID_BAC_R.desired = +pidYaw.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   低速前进模式，带偏航角闭环和角速度内环
 * @param   dt  控制周期（秒）
 * @note    采用串级 PID：外环偏航角 -> 内环角速度 -> 最终轮速
 *          速度基值为 6（较低速）
 */
void Forward_Mode_Low(float dt)
{
    // 角度环
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    // 角速度环（内环）：目标值为角度环输出
    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;   // 陀螺仪原始值转 deg/s
    pidUpdate(&pidRateZ, dt);

    // 四轮速度：基值6，叠加角速度环输出（左右反向）
    PID_FOR_L.desired = 6 - pidRateZ.out;
    PID_FOR_R.desired = 6 + pidRateZ.out;
    PID_BAC_L.desired = 6 - pidRateZ.out;
    PID_BAC_R.desired = 6 + pidRateZ.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   正常前进模式，速度基值20
 */
void Forward_Mode(float dt)
{
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    pidUpdate(&pidRateZ, dt);

    PID_FOR_L.desired = 20 - pidRateZ.out;
    PID_FOR_R.desired = 20 + pidRateZ.out;
    PID_BAC_L.desired = 20 - pidRateZ.out;
    PID_BAC_R.desired = 20 + pidRateZ.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   后退模式，速度基值 -20
 */
void Back_Mode(float dt)
{
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    pidUpdate(&pidRateZ, dt);

    PID_FOR_L.desired = -20 - pidRateZ.out;
    PID_FOR_R.desired = -20 + pidRateZ.out;
    PID_BAC_L.desired = -20 - pidRateZ.out;
    PID_BAC_R.desired = -20 + pidRateZ.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   低速后退模式，速度基值 -4
 */
void Back_Mode_Low(float dt)
{
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    pidUpdate(&pidRateZ, dt);

    PID_FOR_L.desired = -4 - pidRateZ.out;
    PID_FOR_R.desired = -4 + pidRateZ.out;
    PID_BAC_L.desired = -4 - pidRateZ.out;
    PID_BAC_R.desired = -4 + pidRateZ.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   横向右移模式（麦克纳姆轮）
 * @note    四轮速度分配：左前+右后同向，右前+左后反向
 */
void Trans_Mode(float dt)
{
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    pidUpdate(&pidRateZ, dt);

    PID_FOR_L.desired = 20 - pidRateZ.out;
    PID_FOR_R.desired = -20 + pidRateZ.out;
    PID_BAC_L.desired = -20 - pidRateZ.out;
    PID_BAC_R.desired = 20 + pidRateZ.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   横向左移模式（纯左横移，无前进分量）
 * @note    进入本模式的第一拍会自动复位 PID 并锁定当前航向，避免切入瞬间
 *          因积分/微分跳变或目标航向未更新而产生“左前”瞬态。
 *
 *          四轮基准为麦克纳姆 X 形分配（左前/右后同向，右前/左后同向）。
 *          【车头已锁住、但仍向前漂移】= 某一轮的滚轮斜向装反了：
 *          把装反那一轮的宏符号取反即可（一次只翻一个，试到横移最直）。
 *          俯视检查：左前与右后滚轮方向应一致，右前与左后一致，相邻两轮相反。
 */
#define TRANS_LEFT_FL_BASE  (-40)   /* 左前轮基准：装反则改 +20 */
#define TRANS_LEFT_FR_BASE  (+40)   /* 右前轮基准：装反则改 -20 */
#define TRANS_LEFT_BL_BASE  (+40)   /* 左后轮基准：装反则改 -20 */
#define TRANS_LEFT_BR_BASE  (-40)   /* 右后轮基准：装反则改 +20 */

/* 左横移四轮基准速度：运行时变量，默认值=上面宏；方向标定测试时由 main 改写 */
int16_t g_TransLeftBase[4] = { TRANS_LEFT_FL_BASE, TRANS_LEFT_FR_BASE,
                               TRANS_LEFT_BL_BASE, TRANS_LEFT_BR_BASE };

#define CNT_REF  1200.0f   /* 归一化参考一圈计数 */

void Trans_Left_Mode_Run(float dt)
{
    if (Last_Mode_Flag != TRANS_LEFT_MODE) {
        pidRest(pPidObject, 8);   // 清积分/上次误差/输出，消除切入瞬间的跳变
        Angle_Yaw = Angle.yaw;    // 锁定进入时刻的航向，横移过程中车头不转
    }

    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    pidUpdate(&pidRateZ, dt);

    /* 按每轮一圈计数归一化，使四轮物理转速一致，消除横移时的前进/后退分量 */
    PID_FOR_L.desired = (float)g_TransLeftBase[0] * (CNT_LF_turn / CNT_REF) - pidRateZ.out;
    PID_FOR_R.desired = (float)g_TransLeftBase[1] * (CNT_RF_turn / CNT_REF) + pidRateZ.out;
    PID_BAC_L.desired = (float)g_TransLeftBase[2] * (CNT_LB_turn / CNT_REF) - pidRateZ.out;
    PID_BAC_R.desired = (float)g_TransLeftBase[3] * (CNT_RB_turn / CNT_REF) + pidRateZ.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   基于视觉定位的位置闭环模式（取料位置）
 * @param   dt  控制周期
 * @note    位置环输出限幅 ±10，叠加偏航角环，最后分配至四轮。
 *          目标位置固定为 (117, 79) 像素（视觉坐标）。
 */
void Location_Mode(float dt)
{
    // 位置环 PID（X方向、Y方向）
    pidLX.desired = 117;
    pidLY.desired = 79;
    if (LX == 0 || LY == 0) {   // 无效视觉数据时清零目标
        pidLX.desired = 0;
        pidLY.desired = 0;
    }
    pidLX.measured = LX;
    pidLY.measured = LY;
    pidUpdate(&pidLX, dt);
    pidUpdate(&pidLY, dt);

    const float max_speed = 10;
    if (pidLX.out >= max_speed)         pidLX.out = max_speed;
    else if (pidLX.out <= -max_speed)   pidLX.out = -max_speed;
    if (pidLY.out >= max_speed)         pidLY.out = max_speed;
    else if (pidLY.out <= -max_speed)   pidLY.out = -max_speed;

    // 偏航角环（保持航向）
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    // 角速度内环
    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    pidUpdate(&pidRateZ, dt);

    // 四轮速度分配：X方向位移 + Y方向位移 + 旋转项
    PID_FOR_L.desired =  pidLX.out + pidLY.out - pidYaw.out;
    PID_FOR_R.desired =  pidLX.out - pidLY.out + pidYaw.out;
    PID_BAC_L.desired =  pidLX.out - pidLY.out - pidYaw.out;
    PID_BAC_R.desired =  pidLX.out + pidLY.out + pidYaw.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   基于视觉定位的位置闭环模式（放置位置）
 * @note    目标位置 (117, 89)，用于将物料放到指定区域
 */
void Location_Mode_Place(float dt)
{
    pidLX.desired = 117;
    pidLY.desired = 89;
    if (LX == 0 || LY == 0) {
        pidLX.desired = 0;
        pidLY.desired = 0;
    }
    pidLX.measured = LX;
    pidLY.measured = LY;
    pidUpdate(&pidLX, dt);
    pidUpdate(&pidLY, dt);

    const float max_speed = 10;
    if (pidLX.out >= max_speed)         pidLX.out = max_speed;
    else if (pidLX.out <= -max_speed)   pidLX.out = -max_speed;
    if (pidLY.out >= max_speed)         pidLY.out = max_speed;
    else if (pidLY.out <= -max_speed)   pidLY.out = -max_speed;

    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    pidRateZ.desired = pidYaw.out;
    pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
    pidUpdate(&pidRateZ, dt);

    PID_FOR_L.desired = pidLX.out + pidLY.out - pidYaw.out;
    PID_FOR_R.desired = pidLX.out - pidLY.out + pidYaw.out;
    PID_BAC_L.desired = pidLX.out - pidLY.out - pidYaw.out;
    PID_BAC_R.desired = pidLX.out + pidLY.out + pidYaw.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/**
 * @brief   原地旋转模式（无角速度内环）
 * @note    直接根据偏航角误差输出，用于粗调角度
 */
void Revolve_Mode(float dt)
{
    pidYaw.desired = Angle_Yaw;
    pidYaw.measured = Angle.yaw;
    pidUpdate(&pidYaw, dt);

    PID_FOR_L.desired = -pidYaw.out;
    PID_FOR_R.desired = +pidYaw.out;
    PID_BAC_L.desired = -pidYaw.out;
    PID_BAC_R.desired = +pidYaw.out;

    PID_FOR_L.measured = F_L_Speed;
    PID_FOR_R.measured = F_R_Speed;
    PID_BAC_L.measured = B_L_Speed;
    PID_BAC_R.measured = B_R_Speed;

    pidUpdate(&PID_FOR_L, dt);
    pidUpdate(&PID_FOR_R, dt);
    pidUpdate(&PID_BAC_L, dt);
    pidUpdate(&PID_BAC_R, dt);
}

/*==================== 机械臂动作函数 =====================*/

/**
 * @brief   抓取动作序列（根据物料编号执行不同转台角度）
 * @param   Color  1~3，对应转台位置1/2/3
 * @note    流程：转台旋转 -> 升降台下降 -> 夹爪闭合 -> 升降台上升
 *          -> 翻转台收回 -> 下降 -> 松开夹爪 -> 上升 -> 复位
 */
void Catch_Mode(unsigned char Color)
{   
    switch (Color) {
        case 1:
            PWM1_SetCompare3(stage_1 - 20);          // 转台到位置1
            Micorstep_Enable(); DOWN(platform - 1); Delay_ms(500); // 下降
            PWM1_SetCompare2(claw_grab); Delay_ms(500);        // 夹爪夹紧
            Micorstep_Enable(); UP(platform - 1); Delay_ms(500);  // 上升
            PWM1_SetCompare4(could_in + 40); Delay_ms(500);       // 翻转台收回
            Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(500);   // 下降至起始
            PWM1_SetCompare2(claw_free); Delay_ms(1000);       // 松开
            Micorstep_Enable(); UP(start - 1.2); Delay_ms(1000);    // 上升
            PWM1_SetCompare2(claw_free);
            PWM1_SetCompare3(stage_1);
            PWM1_SetCompare4(could_out); break;
        case 2:
            // 同理，转台位置2
            PWM1_SetCompare3(stage_2 - 20);
            Micorstep_Enable(); DOWN(platform - 1); Delay_ms(500);
            PWM1_SetCompare2(claw_grab); Delay_ms(500);
            Micorstep_Enable(); UP(platform - 1); Delay_ms(500);
            PWM1_SetCompare4(could_in + 40); Delay_ms(500);
            Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(500);
            PWM1_SetCompare2(claw_free); Delay_ms(1000);
            Micorstep_Enable(); UP(start - 1.2); Delay_ms(1000);
            PWM1_SetCompare2(claw_free);
            PWM1_SetCompare3(stage_2);
            PWM1_SetCompare4(could_out); break;
        case 3:
            // 转台位置3
            PWM1_SetCompare3(stage_3 - 20);
            Micorstep_Enable(); DOWN(platform - 1); Delay_ms(500);
            PWM1_SetCompare2(claw_grab); Delay_ms(500);
            Micorstep_Enable(); UP(platform - 1); Delay_ms(500);
            PWM1_SetCompare4(could_in + 40); Delay_ms(500);
            Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(500);
            PWM1_SetCompare2(claw_free); Delay_ms(1000);
            Micorstep_Enable(); UP(start - 1.2); Delay_ms(1000);
            PWM1_SetCompare2(claw_free);
            PWM1_SetCompare3(stage_3);
            PWM1_SetCompare4(could_out); break;
    }
}

/**
 * @brief   停止模式，所有轮子速度置0
 */
void Stop_Mode()
{
    Motor_left_front_wheel_SetSpeed(0);
    Motor_right_front_wheel_SetSpeed(0);
    Motor_left_back_wheel_SetSpeed(0);
    Motor_right_back_wheel_SetSpeed(0);
}

/**
 * @brief   将四个轮子的PID输出限幅并写入电机驱动
 * @note    限幅范围 [-1000, 1000]，对应PWM占空比
 */
void Control_Moto()
{
    if (PID_FOR_L.out > 1000)      { PID_FOR_L.out = 1000; }
    else if (PID_FOR_L.out < -1000) { PID_FOR_L.out = -1000; }
    if (PID_FOR_R.out > 1000)      { PID_FOR_R.out = 1000; }
    else if (PID_FOR_R.out < -1000) { PID_FOR_R.out = -1000; }
    if (PID_BAC_L.out > 1000)      { PID_BAC_L.out = 1000; }
    else if (PID_BAC_L.out < -1000) { PID_BAC_L.out = -1000; }
    if (PID_BAC_R.out > 1000)      { PID_BAC_R.out = 1000; }
    else if (PID_BAC_R.out < -1000) { PID_BAC_R.out = -1000; }

    Motor_left_front_wheel_SetSpeed(PID_FOR_L.out);
    Motor_right_front_wheel_SetSpeed(PID_FOR_R.out);
    Motor_left_back_wheel_SetSpeed(PID_BAC_L.out);
    Motor_right_back_wheel_SetSpeed(PID_BAC_R.out);
}

/**
 * @brief   放置动作序列（根据物料编号执行不同转台角度）
 * @param   Color  1~3，对应转台位置
 * @note    用于将已抓取的物料放到目标区域
 */
void Place_Mode(unsigned char Color)
{
    if (Color == 1) {
        PWM1_SetCompare3(stage_1 + 28); Delay_ms(1000);
        PWM1_SetCompare4(could_in + 30); Delay_ms(500);
        Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(1000);
        PWM1_SetCompare2(claw_grab); Delay_ms(1000);
        Micorstep_Enable(); UP(start - 1.2);
        PWM1_SetCompare4(could_out); Delay_ms(1000);

        if (NowStep == 2 || NowStep == 3 || NowStep == 5) {
            Micorstep_Enable(); DOWN(max - 1); Delay_ms(1000); PWM1_SetCompare2(claw_free);
            Micorstep_Enable(); UP(max - 1); Delay_ms(1000);
        } else if (NowStep == 6) {
            Micorstep_Enable(); DOWN(platform - 1); Delay_ms(1000); PWM1_SetCompare2(claw_free);
            Micorstep_Enable(); UP(platform - 1); Delay_ms(1000);
        }
    }
    if (Color == 2) {
        // 类似，转台位置2
        PWM1_SetCompare3(stage_2 + 60); Delay_ms(1000);
        PWM1_SetCompare4(could_in + 25); Delay_ms(500);
        Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(1000);
        PWM1_SetCompare2(claw_grab); Delay_ms(1000);
        Micorstep_Enable(); UP(start - 1.2);
        PWM1_SetCompare4(could_out); Delay_ms(1000);

        if (NowStep == 2 || NowStep == 3 || NowStep == 5) {
            Micorstep_Enable(); DOWN(max - 1); Delay_ms(1000); PWM1_SetCompare2(claw_free);
            Micorstep_Enable(); UP(max - 1); Delay_ms(1000);
        } else if (NowStep == 6) {
            Micorstep_Enable(); DOWN(platform - 1); Delay_ms(1000); PWM1_SetCompare2(claw_free);
            Micorstep_Enable(); UP(platform - 1); Delay_ms(1000);
        }
    }
    if (Color == 3) {
        // 转台位置3
        PWM1_SetCompare3(stage_3 + 28); Delay_ms(1000);
        PWM1_SetCompare4(could_in + 10); Delay_ms(500);
        Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(1000);
        PWM1_SetCompare2(claw_grab); Delay_ms(1000);
        Micorstep_Enable(); UP(start - 1.2);
        PWM1_SetCompare4(could_out); Delay_ms(1000);

        if (NowStep == 2 || NowStep == 3 || NowStep == 5) {
            Micorstep_Enable(); DOWN(max - 1); Delay_ms(1000); PWM1_SetCompare2(claw_free);
            Micorstep_Enable(); UP(max - 1); Delay_ms(1000);
        } else if (NowStep == 6) {
            Micorstep_Enable(); DOWN(platform - 1); Delay_ms(1000); PWM1_SetCompare2(claw_free);
            Micorstep_Enable(); UP(platform - 1); Delay_ms(1000);
        }
    }
}

/**
 * @brief   放置任务调度（配合视觉定位，实现自动寻找目标位置）
 * @param   Place_F  1/2/3 对应红/绿/蓝物料
 * @note    状态机：后退/前进 -> 视觉搜索 -> 位置闭环 -> 执行放置
 */
void Place_ALL(unsigned char Place_F)
{
    static unsigned char Place_Frequence = 0;   // 防止重复触发

    if (Place_Frequence == 2 && NowStep == 5) {
        Place_Frequence = 0;
    }

    // 红色物料
    if (Place_F == 1) {
        if (Place_Frequence == 0) {
            Mode_Flag = BACK_MODE;   // 后退一段距离，进入视觉区域
            Place_Frequence = 1;
        } else if (Place_Frequence == 1 && Mode_Flag == BACK_MODE) {
            Get_wheel_circle(0.7);    // 根据轮子圈数前进/后退固定距离
            LX = 0; LY = 0;            // 清空视觉数据，等待新数据
        } else if (Mode_Flag == STOP_MODE || Place_Frequence == 2) {
            if (P_1 == 0) {
                Serial_TxPacket[0] = 0xAE;   // 发送视觉指令：寻找红色圆形
                P_1 = 1;
            }
        }

        if (LX != 0 && LY != 0 && Place_Over_Red == 0 && Write_Flag == 0xAE) {
            Location_State(117, 89, 1);       // 定位到目标位置（像素坐标117,89）
            if (Location_effective >= 3) {    // 连续3次定位成功
                Mode_Flag = STOP_MODE;
                if (Place_Over_Red == 0) {
                    Location_effective = 0;
                    Serial_TxPacket[0] = 0xFA;   // 关闭视觉识别
                    Place_Red = 1;
                    Place_Over_Red = 1;
                    Place_Frequence = 2;        // 标记完成
                }
            }
        }
    }
    // 绿色物料（Place_F == 2）
    else if (Place_F == 2) {
        if (Place_Frequence == 0) {
            Mode_Flag = STOP_MODE;
            Place_Frequence = 1;
            LX = 0; LY = 0;
        } else if (Mode_Flag == STOP_MODE || Place_Frequence == 2) {
            if (P_2 == 0) {
                Serial_TxPacket[0] = 0xAF;   // 寻找绿色圆形
                P_2 = 1;
            }
        }

        if (LX != 0 && LY != 0 && Place_Over_Green == 0 && Write_Flag == 0xAF) {
            Location_State(117, 89, 1);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Place_Over_Green == 0) {
                    Location_effective = 0;
                    Serial_TxPacket[0] = 0xFA;
                    Place_Green = 1;
                    Place_Over_Green = 1;
                    Place_Frequence = 2;
                }
            }
        }
    }
    // 蓝色物料（Place_F == 3）
    else if (Place_F == 3) {
        if (Place_Frequence == 0) {
            Mode_Flag = FOR_MODE;
            Place_Frequence = 1;
        } else if (Place_Frequence == 1 && Mode_Flag == FOR_MODE) {
            Get_wheel_circle(0.7);
            LX = 0; LY = 0;
        } else if (Mode_Flag == STOP_MODE || Place_Frequence == 2) {
            if (P_3 == 0) {
                Serial_TxPacket[0] = 0xAD;   // 寻找蓝色圆形
                P_3 = 1;
            }
        }

        if (LX != 0 && LY != 0 && Place_Over_Blue == 0 && Write_Flag == 0xAD) {
            Location_State(117, 89, 1);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Place_Over_Blue == 0) {
                    Location_effective = 0;
                    Serial_TxPacket[0] = 0xFA;
                    Place_Blue = 1;
                    Place_Over_Blue = 1;
                    Place_Frequence = 2;
                }
            }
        }
    }
}

/**
 * @brief   放置任务调度（连续模式，根据当前步骤选择不同视觉指令）
 * @param   Place_F  1/2/3 对应红/绿/蓝
 * @note    与 Place_ALL 类似，但视觉指令会根据 NowStep 在圆/方块间切换
 */
void Place_ALL_consient(unsigned char Place_F)
{
    if (Place_F == 1) {
        if (P_1 == 0) {
            if (NowStep == 3)     { Serial_TxPacket[0] = 0xAE; } // 红色圆
            else if (NowStep == 6) { Serial_TxPacket[0] = 0xAA; } // 红色方块（可能为抓取指令）
            P_1 = 1;
        }

        if (LX != 0 && LY != 0 && Place_Over_Red == 0 && (Write_Flag == 0xAE || Write_Flag == 0xAA)) {
            Location_State(117, 89, 1);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Place_Over_Red == 0) {
                    Location_effective = 0;
                    Serial_TxPacket[0] = 0xFA;
                    Place_Red = 1;
                    Place_Over_Red = 1;
                }
            }
        }
    }
    else if (Place_F == 2) {
        if (P_2 == 0) {
            if (NowStep == 3)     { Serial_TxPacket[0] = 0xAF; } // 绿色圆
            else if (NowStep == 6) { Serial_TxPacket[0] = 0xAB; } // 绿色方块
            P_2 = 1;
        }

        if (LX != 0 && LY != 0 && Place_Over_Green == 0 && (Write_Flag == 0xAF || Write_Flag == 0xAB)) {
            Location_State(117, 89, 1);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Place_Over_Green == 0) {
                    Location_effective = 0;
                    Serial_TxPacket[0] = 0xFA;
                    Place_Green = 1;
                    Place_Over_Green = 1;
                }
            }
        }
    }
    if (Place_F == 3) {
        if (P_3 == 0) {
            if (NowStep == 3)     { Serial_TxPacket[0] = 0xAD; } // 蓝色圆
            else if (NowStep == 6) { Serial_TxPacket[0] = 0xAC; } // 蓝色方块
            P_3 = 1;
        }

        if (LX != 0 && LY != 0 && Place_Over_Blue == 0 && (Write_Flag == 0xAD || Write_Flag == 0xAC)) {
            Location_State(117, 89, 1);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Place_Over_Blue == 0) {
                    Location_effective = 0;
                    Serial_TxPacket[0] = 0xFA;
                    Place_Blue = 1;
                    Place_Over_Blue = 1;
                }
            }
        }
    }
}

/**
 * @brief   自动放置函数（循环执行三次放置动作）
 * @note    按顺序放置三种物料，并在每次放置后根据视觉数据调整运动方向
 */
void Place_Sthing()
{
    static unsigned char Frequence = 0;
    Frequence %= 3;
    Frequence++;
    Place_Mode(Frequence);
    Delay_ms(1000);

    BuzzerFlag = 1;   // 蜂鸣器提示

    if (Frequence == 1) {
        pidRest(pPidObject, 8);    // 复位所有PID
        LX = 0; LY = 0;
        Find2 = 1; Find1 = 0; Find3 = 0;
        if (Serial_TxPacket[2] <= Serial_TxPacket[1])
            { Mode_Flag = BACK_MODE_Low; }
        else
            { Mode_Flag = FOR_MODE_LOW; }
    }
    else if (Frequence == 2) {
        pidRest(pPidObject, 8);
        LX = 0; LY = 0;
        Find3 = 1; Find1 = 0; Find2 = 0;
        if (Serial_TxPacket[3] <= Serial_TxPacket[2])
            { Mode_Flag = BACK_MODE_Low; }
        else
            { Mode_Flag = FOR_MODE_LOW; }
    }
    else if (Frequence == 3) {
        pidRest(pPidObject, 8);
        LX = 0; LY = 0;
        Find3 = 0; Find1 = 0; Find2 = 0;
        if (NowStep == 3 || NowStep == 6)
            { Mode_Flag = BACK_MODE; }   // 离开放置区
    }
}

/**
 * @brief   抓取任务调度（根据颜色编号执行视觉搜索与定位）
 * @param   Catch_F  1/2/3 对应红/绿/蓝
 * @note    与放置类似，但目标位置为 (120, 89)，且最终执行抓取动作
 */
void Catch_All(unsigned char Catch_F)
{
    if (Catch_F == 1) {
        if (C_1 == 0) {
            Serial_TxPacket[0] = 0xAA;   // 寻找红色方块
            C_1 = 1;
        }

        if (LX != 0 && LY != 0 && Catch_Over_Red == 0 && Write_Flag == 0xAA) {
            Location_State(120, 89, 4);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Catch_Over_Red == 0) {
                    Location_effective = 0;
                    Catch_Red = 1;
                    Catch_Over_Red = 1;
                }
            }
        }
    }

    if (Catch_F == 2) {
        if (C_2 == 0) {
            Serial_TxPacket[0] = 0xAB;   // 寻找绿色方块
            C_2 = 1;
        }

        if (LX != 0 && LY != 0 && Catch_Over_Green == 0 && Write_Flag == 0xAB) {
            Location_State(120, 89, 4);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Catch_Over_Green == 0) {
                    Location_effective = 0;
                    Catch_Green = 1;
                    Catch_Over_Green = 1;
                }
            }
        }
    }

    if (Catch_F == 3) {
        if (C_3 == 0) {
            Serial_TxPacket[0] = 0xAC;   // 寻找蓝色方块
            C_3 = 1;
        }

        if (LX != 0 && LY != 0 && Catch_Over_Blue == 0 && Write_Flag == 0xAC) {
            Location_State(120, 89, 4);
            if (Location_effective >= 3) {
                Mode_Flag = STOP_MODE;
                if (Catch_Over_Blue == 0) {
                    Location_effective = 0;
                    Catch_Blue = 1;
                    Catch_Over_Blue = 1;
                }
            }
        }
    }
}

/**
 * @brief   抓取动作序列（用于Catch_Sthing，与Catch_Mode基本相同但参数略有调整）
 * @param   Color  1~3，对应转台位置
 */
void Catch_Mode_St(unsigned char Color)
{
    switch (Color) {
        case 1:
            PWM1_SetCompare3(stage_1 - 20);
            Micorstep_Enable(); DOWN(max - 1); Delay_ms(500);
            PWM1_SetCompare2(claw_grab); Delay_ms(500);
            Micorstep_Enable(); UP(max - 1); Delay_ms(500);
            PWM1_SetCompare4(could_in + 40); Delay_ms(500);
            Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(500);
            PWM1_SetCompare2(claw_free); Delay_ms(1000);
            Micorstep_Enable(); UP(start - 1.2); Delay_ms(1000);
            PWM1_SetCompare2(claw_free);
            PWM1_SetCompare3(stage_1);
            PWM1_SetCompare4(could_out); break;
        case 2:
            // 类似
            PWM1_SetCompare3(stage_2 - 20);
            Micorstep_Enable(); DOWN(max - 1); Delay_ms(500);
            PWM1_SetCompare2(claw_grab); Delay_ms(500);
            Micorstep_Enable(); UP(max - 1); Delay_ms(500);
            PWM1_SetCompare4(could_in + 40); Delay_ms(500);
            Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(500);
            PWM1_SetCompare2(claw_free); Delay_ms(1000);
            Micorstep_Enable(); UP(start - 1.2); Delay_ms(1000);
            PWM1_SetCompare2(claw_free);
            PWM1_SetCompare3(stage_2);
            PWM1_SetCompare4(could_out); break;
        case 3:
            PWM1_SetCompare3(stage_3 - 20);
            Micorstep_Enable(); DOWN(max - 1); Delay_ms(500);
            PWM1_SetCompare2(claw_grab); Delay_ms(500);
            Micorstep_Enable(); UP(max - 1); Delay_ms(500);
            PWM1_SetCompare4(could_in + 40); Delay_ms(500);
            Micorstep_Enable(); DOWN(start - 1.2); Delay_ms(500);
            PWM1_SetCompare2(claw_free); Delay_ms(1000);
            Micorstep_Enable(); UP(start - 1.2); Delay_ms(1000);
            PWM1_SetCompare2(claw_free);
            PWM1_SetCompare3(stage_3);
            PWM1_SetCompare4(could_out); break;
    }
}

/**
 * @brief   自动抓取函数（循环执行三次抓取动作）
 * @note    与 Place_Sthing 类似，但执行抓取序列，并根据视觉数据调整运动方向
 */
void Catch_Sthing()
{
    static unsigned char Frequence = 0;
    Frequence %= 3;
    Frequence++;
    Catch_Mode_St(Frequence);
    Delay_ms(1000);

    if (Frequence == 1) {
        pidRest(pPidObject, 8);
        LX = 0; LY = 0;
        Catch2 = 1; Catch1 = 0; Catch3 = 0;
        if (Serial_TxPacket[2] <= Serial_TxPacket[1]) { Mode_Flag = BACK_MODE_Low; }
        else { Mode_Flag = FOR_MODE_LOW; }
    }
    if (Frequence == 2) {
        pidRest(pPidObject, 8);
        LX = 0; LY = 0;
        Catch3 = 1; Catch2 = 0; Catch1 = 0;
        if (Serial_TxPacket[3] <= Serial_TxPacket[2]) { Mode_Flag = BACK_MODE_Low; }
        else { Mode_Flag = FOR_MODE_LOW; }
    }
    if (Frequence == 3) {
        pidRest(pPidObject, 8);
        LX = 0; LY = 0;
        Catch3 = 0; Catch2 = 0; Catch1 = 0;
        Mode_Flag = BACK_MODE;
    }
}

/**
 * @brief   视觉定位状态控制（调用自动寻迹运动）
 * @param   Designed_LX  目标X像素
 * @param   Designed_LY  目标Y像素
 * @param   erro         定位允许误差（像素）
 * @note    内部通过 CamCtrl_AutoCenter 计算移动距离 Location_distance，
 *          然后调用 Get_wheel_circle 执行位移，实现闭环逼近。
 */
void Location_State(float Designed_LX, float Designed_LY, uint8_t erro)
{
    cam_ctrl.CAM_CENTER_X = Designed_LX;
    cam_ctrl.CAM_CENTER_Y = Designed_LY;
    cam_ctrl.MOVE_TOLERANCE = erro;

    static uint16_t LOCATION_Delay = 0;
    if (Mode_Flag == STOP_MODE) {
        Location_distance = 0;
        LOCATION_Delay++;
        if (LOCATION_Delay >= 90) {
            CamCtrl_AutoCenter(&cam_ctrl, LX, LY);   // 计算需要移动的距离
            LOCATION_Delay = 0;
        }
    }
    Get_wheel_circle(Location_distance);   // 执行移动
}

extern unsigned char Catch_Frequency;

/*==================== 完整任务状态机 =====================*/

/**
 * @brief   完整搬运任务状态机
 * @note    由外部循环调用，逐步执行：
 *          前进 -> 横向平移 -> 抓取 -> 后退 -> 旋转 -> 前行 -> 旋转 -> 放置 -> ...
 *          包含两圈（两次抓取/放置循环）
 */
void LineMove_Mode(void)
{
    volatile static uint8_t state = MOVE_11;   // 状态机当前状态
    static uint16_t delayFlag = 0;             // 延时计数器

    switch (state) {
        case start_1: {
            Mode_Flag = STOP_MODE;
            break;
        }

        case MOVE_11:   // 从起点出发，前进一段
        {
            Get_wheel_circle(0.9);
            if (Mode_Flag == STOP_MODE) {
                delayFlag++;
                if (delayFlag >= 10) {
                    Mode_Flag = FOR_MODE_LOW;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = MOVE_13;
                }
            }
            break;
        }

        case MOVE_13:   // 继续前进并检测二维码，准备横向平移
        {
            Get_wheel_circle(2.78 + 2.9 + 0.45);
            static uint8_t i = 0;
            if (Write_Flag == 0xcb && i == 0) {   // 检测到二维码
                i++;
                Mode_Flag = FOR_MODE;
            }
            if (Mode_Flag == STOP_MODE) {
                delayFlag++;
                if (delayFlag >= 10) {
                    Mode_Flag = TRANS_MODE;       // 横向右移
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = MOVE_17;
                }
            }
            break;
        }

        case MOVE_17:   // 横向移动到抓取位置
        {
            Get_wheel_circle(0.48);
            if (Mode_Flag == STOP_MODE && Write_Flag == 0x02) {
                delayFlag++;
                if (delayFlag >= 100) {
                    BuzzerFlag = 1;
                    NowStep++;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = GRASP_70;
                }
            }
            break;
        }

        case GRASP_70:   // 视觉定位并抓取
        {
            if (NowStep == 1)
                { Serial_TxPacket[0] = 0XB1; }   // 第一圈抓取指令
            else if (NowStep == 4)
                { Serial_TxPacket[0] = 0XBA; }   // 第二圈抓取指令

            Mode_Flag = STOP_MODE;
            Catch_Flag = 1;
            BuzzerFlag = 1;

            if (Catch_Flag == 1 && Mode_Flag == STOP_MODE && Catch_Frequency >= 3) {
                delayFlag++;
                if (delayFlag >= 50) {
                    Catch_Flag = 0;
                    BuzzerFlag = 1;
                    Mode_Flag = BACK_MODE;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = MOVE_14;
                }
            }
            break;
        }

        case MOVE_14:   // 后退并准备旋转
        {
            Serial_TxPacket[0] = 0xFA;
            Get_wheel_circle(1.69);
            if (Mode_Flag == STOP_MODE && Write_Flag == 0xFA) {
                delayFlag++;
                if (delayFlag >= 100) {
                    BuzzerFlag = 1;
                    Angle_Yaw = 90;
                    Mode_Flag = REVOLVE_MODE;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = MOVE_15;
                }
            }
            break;
        }

        case MOVE_15:   // 旋转90°
        {
            static uint8_t revolve_num = 0;
            if ((Angle.yaw >= 90 - 2 && Angle.yaw <= 90 + 2) || revolve_num == 1) {
                revolve_num = 1;
                Mode_Flag = STOP_MODE;
                delayFlag++;
                if (delayFlag >= 50) {
                    Mode_Flag = FOR_MODE;
                    revolve_num = 0;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = MOVE_16;
                }
            }
            break;
        }

        case MOVE_16:   // 前进至十字路口
        {
            Get_wheel_circle(7.7);
            if (Mode_Flag == STOP_MODE) {
                delayFlag++;
                if (delayFlag >= 100) {
                    Angle_Yaw = 177;
                    Mode_Flag = REVOLVE_MODE;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = MOVE_18;
                }
            }
            break;
        }

        case MOVE_18:   // 旋转到177°（接近180°）
        {
            static uint8_t revolve_num = 0;
            if ((Angle.yaw >= 177 - 2 && Angle.yaw <= 177 + 2) || revolve_num == 1) {
                revolve_num = 1;
                Mode_Flag = STOP_MODE;
                delayFlag++;
                if (delayFlag >= 100) {
                    Mode_Flag = STOP_MODE;
                    revolve_num = 0;
                    CamCtrl_Init(&cam_ctrl);
                    NowStep++;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = GRASP_71;
                }
            }
            break;
        }

        case GRASP_71:   // 第一次放置与抓取（第一圈）
        {
            // 放置流程
            if (Find1 == 1)           { Place_ALL(Serial_TxPacket[1]); }
            else if (Find2 == 1)      { Place_ALL(Serial_TxPacket[2]); }
            else if (Find3 == 1)      { Place_ALL(Serial_TxPacket[3]); }

            // 抓取流程（在放置完成后）
            if (Catch1 == 1 && Find1 == 0 && Find2 == 0 && Find3 == 0) {
                if (Catch_object == 0) {
                    cam_ctrl.pixel_to_cm_x = 0.002f;
                    cam_ctrl.pixel_to_cm_y = 0.003f;
                    if (Serial_TxPacket[1] <= Serial_TxPacket[3]) { Mode_Flag = BACK_MODE_Low; }
                    else { Mode_Flag = FOR_MODE_LOW; }
                    Catch_object = 1;
                }
                Catch_All(Serial_TxPacket[1]);
            }
            else if (Catch2 == 1) { Catch_All(Serial_TxPacket[2]); }
            else if (Catch3 == 1) { Catch_All(Serial_TxPacket[3]); }

            // 所有抓取完成，开始前进
            if (Catch3 == 0 && Catch2 == 0 && Catch1 == 0) {
                if (Serial_TxPacket[3] == 1)     { Get_wheel_circle(2.85); }
                else if (Serial_TxPacket[3] == 2) { Get_wheel_circle(3.55); }
                else if (Serial_TxPacket[3] == 3) { Get_wheel_circle(4.25); }
                if (Mode_Flag == STOP_MODE) {
                    delayFlag++;
                    if (delayFlag >= 100) {
                        Angle_Yaw = 90;
                        Mode_Flag = REVOLVE_MODE;
                        delayFlag = 0;
                        pidRest(pPidObject, 8);
                        state = MOVE_19;
                    }
                }
            }
            break;
        }

        case MOVE_19:   // 旋转90°，准备第二圈放置
        {
            static uint8_t revolve_num = 0;
            if ((Angle.yaw >= 90 - 2 && Angle.yaw <= 90 + 2) || revolve_num == 1) {
                revolve_num = 1;
                Mode_Flag = STOP_MODE;
                delayFlag++;
                if (delayFlag >= 50) {
                    Mode_Flag = BACK_MODE;
                    revolve_num = 0;
                    NowStep++;
                    // 复位放置标志
                    Find1 = 1; Find2 = 0; Find3 = 0;
                    P_1 = 0; P_2 = 0; P_3 = 0;
                    Place_Over_Red = 0; Place_Over_Blue = 0; Place_Over_Green = 0;

                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = GRASP_72;
                }
            }
            break;
        }

        case GRASP_72:   // 第二圈放置和抓取（连续模式）
        {
            // 放置（连续模式）
            if (Find1 == 1)           { Place_ALL_consient(Serial_TxPacket[1]); }
            else if (Find2 == 1)      { Place_ALL_consient(Serial_TxPacket[2]); }
            else if (Find3 == 1)      { Place_ALL_consient(Serial_TxPacket[3]); }

            // 放置完成后，前进
            if (Find1 == 0 && Find2 == 0 && Find3 == 0) {
                if (Serial_TxPacket[3] == 1)     { Get_wheel_circle(2.85); }
                else if (Serial_TxPacket[3] == 2) { Get_wheel_circle(3.55); }
                else if (Serial_TxPacket[3] == 3) { Get_wheel_circle(4.25); }
                if (Mode_Flag == STOP_MODE) {
                    delayFlag++;
                    if (delayFlag >= 100) {
                        Angle_Yaw = 3;
                        Mode_Flag = REVOLVE_MODE;
                        delayFlag = 0;
                        pidRest(pPidObject, 8);
                        state = MOVE_20;
                    }
                }
            }
            break;
        }

        case MOVE_20:   // 旋转回初始方向
        {
            static uint8_t revolve_num = 0;
            if ((Angle.yaw >= 3 - 2 && Angle.yaw <= 3 + 2) || revolve_num == 1) {
                revolve_num = 1;
                Mode_Flag = STOP_MODE;
                delayFlag++;
                if (delayFlag >= 100) {
                    Mode_Flag = BACK_MODE;
                    pid_Para_Init_Location(1);
                    revolve_num = 0;
                    NowStep++;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = GRASP_73;
                }
            }
            break;
        }

        case GRASP_73:   // 第二圈抓取准备（后退并切换物料顺序）
        {
            Get_wheel_circle(2.2);
            if (Mode_Flag == STOP_MODE) {
                delayFlag++;
                if (delayFlag >= 300) {
                    // 重置标志，准备第二圈抓取
                    Find1 = 1; Find2 = 0; Find3 = 0;
                    P_1 = 0; P_2 = 0; P_3 = 0;
                    Place_Over_Red = 0; Place_Over_Blue = 0; Place_Over_Green = 0;

                    Catch1 = 1; Catch2 = 0; Catch3 = 0;
                    Catch_object = 0;
                    C_1 = 0; C_2 = 0; C_3 = 0;
                    Catch_Over_Red = 0; Catch_Over_Blue = 0; Catch_Over_Green = 0;
                    Catch_Frequency = 0;

                    // 物料顺序更新（可能来自二维码数据）
                    Serial_TxPacket[1] = Serial_TxPacket[4];
                    Serial_TxPacket[2] = Serial_TxPacket[5];
                    Serial_TxPacket[3] = Serial_TxPacket[6];

                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    if (NowStep == 4) {
                        state = GRASP_70;   // 回到抓取状态（第二圈）
                    } else if (NowStep == 7) {
                        state = MOVE_21;
                        Mode_Flag = BACK_MODE;
                    }
                }
            }
            break;
        }

        case MOVE_21:   // 返回起点
        {
            Get_wheel_circle(2.78 + 2.9 + 0.35);
            if (Mode_Flag == STOP_MODE) {
                delayFlag++;
                if (delayFlag >= 10) {
                    Mode_Flag = TRANS_MODE;
                    delayFlag = 0;
                    pidRest(pPidObject, 8);
                    state = MOVE_22;
                }
            }
            break;
        }

        case MOVE_22:   // 回到起点，任务结束
        {
            Get_wheel_circle(0.9);
            break;
        }

        default:
            state = 0;
            break;
    }
}