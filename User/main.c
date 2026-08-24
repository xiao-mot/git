#include "stm32f10x.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "String_To_Num.h"//字符串转换为数字相关函数
#include "Delay.h"
#include "Encoder.h"
#include "PID.h"
#include "PWM.h"
#include "Timer4.h"
#include "stdio.h"//支持print
#include "mpu6050.h"
#include "Init.h"
#include "ALL_DATA.h"
#include "Control.h"
#include <stdint.h>

/*
 * 主函数常用接口索引
 * 以下原型均来自现有 .h 文件，并且已在对应 .c 文件中找到实现。
 * 这里重复声明只用于学习和查阅，不会创建新函数。
 */

/* 系统初始化：ALL_Init 已经调用电机、编码器、串口和 TIM6 初始化函数。 */
void ALL_Init(void);          // 初始化整车外设和控制参数，程序启动时调用一次
void Motor_Init(void);        // 初始化四轮方向 GPIO 和 TIM8 PWM，ALL_Init 已调用
void Encoder_Init(void);      // 初始化四路编码器，ALL_Init 已调用
void TIM6_Init(void);         // 启动 1 ms 定时中断，ALL_Init 已调用

/* 基础调试接口。 */
void Buzzer_bb(void);         // 蜂鸣器提示
void Delay_ms(uint32_t ms);   // 毫秒阻塞延时，不能放进高频控制中断
uint8_t Key_GetNum(void);     // 读取按键编号

/* 四轮开环输出：参数正负控制方向，绝对值对应 PWM 比较值。 */
void Motor_left_front_wheel_SetSpeed(int16_t lf_speed);    // 左前轮
void Motor_right_front_wheel_SetSpeed(int16_t rfront_speed);// 右前轮
void Motor_left_back_wheel_SetSpeed(int16_t lb_speed);     // 左后轮
void Motor_right_back_wheel_SetSpeed(int16_t rb_speed);    // 右后轮

/* 编码器反馈：Get_Speed 统一更新四轮速度，Get 函数读取单轮计数。 */
void Get_Speed(void);
int16_t left_front_Encoder_Get(void);
int16_t right_front_Encoder_Get(void);
int16_t left_back_Encoder_Get(void);
int16_t right_back_Encoder_Get(void);

/*
 * PID 运动控制：Pid_Control_Trans 由 TIM6 每 5 ms 调度。
 * 主循环通常只修改 Mode_Flag 和目标值，不应无周期地反复调用这些 PID 函数。
 */
void Pid_Control_Trans(void);      // 按 Mode_Flag 选择运动模式并输出到电机
void Forward_Mode(float dt);       // 前进闭环
void Forward_Mode_Low(float dt);   // 低速前进闭环
void Back_Mode(float dt);          // 后退闭环
void Back_Mode_Low(float dt);      // 低速后退闭环
void Trans_Mode(float dt);         // 向右横移闭环
void Trans_Left_Mode_Run(float dt);    // 向左横移闭环
void Revolve_Mode(float dt);       // 按 Angle_Yaw 目标转向
void Revolve_Mode_150(float dt);   // 工程保留的转向模式
void Revolve_Mode_0(float dt);     // 工程保留的转向模式
void Stop_Mode(void);              // 四轮 PWM 置零
void Control_Moto(void);           // PID 输出限幅后写入四个电机
void LineMove_Mode(void);          // 当前比赛流程状态机

/* 现有串口接口：只重复声明已有函数，不在这里实现蓝牙协议。 */
uint8_t Serial_GetRxFlag(void);     // 查询现有串口接收完成标志
void Serial_SendPacket(void);       // 发送现有视觉/任务数据包
void user_printf(const char *format, ...); // 通过 USART2 输出调试文本

unsigned char QrCode = 0; // 二维码标志位
unsigned char BuzzerFlag = 0;
unsigned char Catch_Frequency = 0;//抓取的次数




int main(void)
{


    /* ---------- 正常主流程 ---------- */
    ALL_Init();

    Mode_Flag = STOP_MODE;
    Stop_Mode();

    Delay_ms(500);        // 等 MPU6050 姿态稳定（航向来自 MPU6050）
    /* HWT101 备选（采购后启用）：等待 HWT101 产生有效航向数据
    while (new_data_received == 0U) { }
    */

   

    pidRest(pPidObject, 8);

    Angle_Yaw = Angle.yaw;        // 保持当前方向（MPU6050 解算航向）
    //Angle_Yaw = global_angle;   // HWT101 备选：保持当前方向
    Mode_Flag = TRANS_LEFT_MODE;  // 进入左横移模式，由 TIM6 中断调度 Trans_Left_Mode_Run
    Mode_Flag = REVOLVE_MODE_150;

    while (1)
    {
     
     
    }
}

