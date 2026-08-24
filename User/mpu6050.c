/**
 * @file    mpu6050.c
 * @brief   MPU6050 六轴惯性传感器驱动及姿态解算模块
 * @note    包含初始化、数据读取、滤波、零偏校准、四元数姿态解算等功能
 *          适用于物流搬运小车等需要姿态感知的应用
 */

#include "mpu6050.h"

/*==================== 寄存器地址宏定义 ====================*/
#define SMPLRT_DIV      0x19    // 采样率分频器，典型值 0x07 对应 125Hz
#define CONFIGL         0x1A    // 数字低通滤波器配置，典型值 0x06 对应 5Hz
#define GYRO_CONFIG     0x1B    // 陀螺仪量程配置，0x18 对应 ±2000deg/s
#define ACCEL_CONFIG    0x1C    // 加速度计量程及滤波配置，0x09 对应 ±2G 且 DLPF 为 5Hz
#define ACCEL_ADDRESS   0x3B    // 加速度计数据起始地址
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_ADDRESS    0x43    // 陀螺仪数据起始地址
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define PWR_MGMT_1      0x6B    // 电源管理寄存器，0x00 表示内部时钟使能
#define WHO_AM_I        0x75    // 器件 ID 寄存器，读出的值应为 0x68

#define MPU6050_PRODUCT_ID     0x68    // MPU6050 的固定 ID
#define MPU6052C_PRODUCT_ID    0x72    // 另一型号 ID（未使用）
#define SUCCESS    0
#define FAILED     1
#define MPU6050_ADDRESS  0xD0   // 器件地址（7位地址 0x68 左移一位得到 0xD0）

/*==================== 全局变量定义 ====================*/
_st_Mpu  MPU6050;       // 原始数据（加速度和陀螺仪）结构体
_st_AngE Angle;         // 解算后的姿态角（pitch, roll, yaw）

int16_t MpuOffset[6] = {0};    // 六轴零偏校正值（顺序：accX, accY, accZ, gyroX, gyroY, gyroZ）

/**
 * @brief   将结构体 MPU6050 的地址映射为有符号16位整数数组指针
 * @note    利用此指针可连续访问加速度和陀螺仪共6个轴的数据
 */
static volatile int16_t *pMpu = (int16_t *)&MPU6050;

/*==================== 卡尔曼滤波函数（一维） =====================*/
/**
 * @brief   一阶卡尔曼滤波（用于加速度数据平滑）
 * @param   ekf  指向滤波器结构体的指针
 * @param   input 当前输入值
 * @note    结构体中的参数：Q 过程噪声协方差，R 测量噪声协方差，
 *          LastP 上次误差协方差，Now_P 本次预测误差协方差，
 *          Kg 卡尔曼增益，out 滤波输出值
 */
void kalman_1(struct _1_ekf_filter *ekf, float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;
}

/*==================== 复位 MPU6050 =====================*/
/**
 * @brief   软件复位 MPU6050（将电源管理寄存器置 0x80）
 * @retval  SUCCESS 或 FAILED
 */

int8_t mpu6050_rest(void)
{
    if (IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80) == FAILED)
        return FAILED;
    Delay_ms(20);
    return SUCCESS;
}

/*==================== MPU6050 初始化 =====================*/
/**
 * @brief   初始化 MPU6050 芯片，配置采样率、滤波器、量程等
 * @retval  SUCCESS 或 FAILED
 * @note    若读取 WHO_AM_I 不是 0x68，则初始化失败
 */
int8_t MpuInit(void)
{
    uint8_t date = SUCCESS;
    do
    {
        // 写入各配置寄存器，采用累加方式检测是否全部成功
        date = IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);
        Delay_ms(30);
        date += IIC_Write_One_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x02);
        date += IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);
        date += IIC_Write_One_Byte(MPU6050_ADDRESS, CONFIGL, 0x03);
        date += IIC_Write_One_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);
        date += IIC_Write_One_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);
    } while (date != SUCCESS);  // 若失败则重试

    date = IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75);
    if (date != MPU6050_PRODUCT_ID)
        return FAILED;

    // 可从 Flash 中读取保存的零偏值（此处注释掉，改为运行时校准）
    // FLASH_read(MpuOffset,6);
    return SUCCESS;
}

/*==================== 读取原始数据的宏 =====================*/
#define Acc_Read()  IIC_read_Bytes(MPU6050_ADDRESS, 0X3B, buffer, 6)
#define Gyro_Read() IIC_read_Bytes(MPU6050_ADDRESS, 0x43, &buffer[6], 6)

/*==================== 获取并滤波数据 =====================*/
/**
 * @brief   读取加速度和陀螺仪原始数据，进行零偏校正和滤波
 * @note    加速度数据使用一维卡尔曼滤波，陀螺仪数据使用一阶低通滤波
 *          滤波后的数据直接更新到全局结构体 MPU6050 中
 */
void MpuGetData(void)
{
    uint8_t i;
    uint8_t buffer[12];

    Acc_Read();   // 读取加速度计 6 个字节
    Gyro_Read();  // 读取陀螺仪 6 个字节

    for (i = 0; i < 6; i++)
    {
        // 将两个字节拼接为有符号16位整数，并减去零偏
        pMpu[i] = (((int16_t)buffer[i << 1] << 8) | buffer[(i << 1) + 1]) - MpuOffset[i];

        if (i < 3)  // 加速度轴（X, Y, Z）
        {
            // 使用三个独立的卡尔曼滤波器，分别处理三个轴
            static struct _1_ekf_filter ekf[3] = {
                {0.02, 0, 0, 0, 0.001, 0.543},
                {0.02, 0, 0, 0, 0.001, 0.543},
                {0.02, 0, 0, 0, 0.001, 0.543}
            };
            kalman_1(&ekf[i], (float)pMpu[i]);
            pMpu[i] = (int16_t)ekf[i].out;
        }
        else        // 陀螺仪轴（X, Y, Z）
        {
            uint8_t k = i - 3;
            const float factor = 0.15f;      // 一阶低通滤波系数
            static float tBuff[3];           // 上一时刻的滤波值缓存

            // 一阶低通：out = (1-factor)*last + factor*new
            pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;
        }
    }
}

/*==================== 零偏校准 =====================*/
/**
 * @brief   校准 MPU6050 的零偏值
 * @note    需保证芯片静止，先判断陀螺仪稳定（相邻读数变化小于5），
 *          然后取 256 组数据的平均值作为偏移，并写入 MpuOffset 数组。
 *          校准过程中会禁用 TIM6 中断，避免干扰。
 */
void MpuGetOffset(void)
{
    int32_t buffer[6] = {0};
    int16_t i;
    uint8_t k = 30;
    const int8_t MAX_GYRO_QUIET = 5;
    const int8_t MIN_GYRO_QUIET = -5;
    int16_t LastGyro[3] = {0};
    int16_t ErrorGyro[3];

    // 清空偏移数组，并将加速度 Z 轴预设为 8192（即 1g 对应的值）
    memset(MpuOffset, 0, 12);
    MpuOffset[2] = 8192;

    // 关闭定时器中断，防止在校准过程中发生中断干扰数据采集
    TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);

    // 等待陀螺仪数据稳定：连续采样，直到相邻三次采样差值都在 ±5 以内
    while (k--)  // 最多尝试30次
    {
        do
        {
            Delay_ms(10);
            MpuGetData();
            for (i = 0; i < 3; i++)
            {
                ErrorGyro[i] = pMpu[i + 3] - LastGyro[i];
                LastGyro[i] = pMpu[i + 3];
            }
        } while ((ErrorGyro[0] > MAX_GYRO_QUIET) || (ErrorGyro[0] < MIN_GYRO_QUIET) ||
                 (ErrorGyro[1] > MAX_GYRO_QUIET) || (ErrorGyro[1] < MIN_GYRO_QUIET) ||
                 (ErrorGyro[2] > MAX_GYRO_QUIET) || (ErrorGyro[2] < MIN_GYRO_QUIET));
    }

    // 采集 356 组数据，丢弃前 100 组，用后 256 组进行平均
    for (i = 0; i < 356; i++)
    {
        MpuGetData();
        if (100 <= i)   // 从第100组开始累加
        {
            uint8_t k;
            for (k = 0; k < 6; k++)
            {
                buffer[k] += pMpu[k];
            }
        }
    }

    // 计算平均值（右移8位等效于除以256）
    for (i = 0; i < 6; i++)
    {
        MpuOffset[i] = buffer[i] >> 8;
    }

    // 恢复定时器中断
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    // 可将校准结果保存到 Flash（此处注释）
    // FLASH_write(MpuOffset,6);
}

/*==================== 快速平方根倒数算法 =====================*/
/**
 * @brief   计算 1/sqrt(x) 的快速算法（Quake III 中的经典实现）
 * @param   number 输入浮点数
 * @return  1/sqrt(number)
 * @note    用于归一化向量，比标准 sqrt 快很多
 */
float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));   // 一次牛顿迭代
    return y;
}

/*==================== 姿态解算相关常量 =====================*/
const float M_PI = 3.1415926535f;
const float RtA = 57.2957795f;          // 弧度转角度
const float AtR = 0.0174532925f;        // 角度转弧度
const float Gyro_G = 0.03051756f * 2;   // 陀螺仪原始值转角速度(deg/s)，对应 ±2000°/s
const float Gyro_Gr = 0.0005326f * 2;   // 陀螺仪原始值转角速度(rad/s)

static float NormAcc;                   // 加速度归一化后模长
Quaternion NumQ = {1, 0, 0, 0};         // 四元数，初始为单位四元数
volatile struct V GyroIntegError = {0}; // 陀螺仪积分误差（用于互补滤波）

/*==================== 姿态解算函数 =====================*/
/**
 * @brief   根据 MPU6050 数据解算姿态角（pitch, roll, yaw）
 * @param   pMpu   指向原始数据结构体的指针
 * @param   pAngE  输出姿态角结构体指针
 * @param   dt     采样周期（秒）
 * @note    使用四元数 + 互补滤波（Mahony 滤波器变种），
 *          利用加速度计修正陀螺仪漂移，输出欧拉角。
 *          偏航角（yaw）默认只由陀螺仪积分，若定义了 YAW_GYRO 宏则采用磁力计融合（但此处未实现）。
 */
void GetAngle(const _st_Mpu *pMpu, _st_AngE *pAngE, float dt)
{
    volatile struct V Gravity, Acc, Gyro, AccGravity;

    static float KpDef = 0.5f;      // 比例系数
    static float KiDef = 0.0001f;   // 积分系数
    // static float KiDef = 0.00001f;  // 备选积分系数

    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 1. 根据当前四元数计算重力方向（在机体坐标系中的表示）
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // 2. 将加速度计读数归一化
    NormAcc = Q_rsqrt(squa(pMpu->accX) + squa(pMpu->accY) + squa(pMpu->accZ));
    Acc.x = pMpu->accX * NormAcc;
    Acc.y = pMpu->accY * NormAcc;
    Acc.z = pMpu->accZ * NormAcc;

    // 3. 计算加速度与重力方向的误差（叉积）
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    // 4. 误差积分（用于消除陀螺仪零偏）
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    // 5. 使用误差修正陀螺仪测量值（比例+积分）
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

    // 6. 四元数微分方程（一阶龙格-库塔）
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 7. 四元数归一化（保证单位四元数）
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // 8. 从四元数计算欧拉角（pitch, roll, yaw）
    {
#ifdef YAW_GYRO
        // 若定义了此宏，则使用磁力计或其他方式融合的偏航角（此处未实现）
        *(float *)pAngE = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3,
                                 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA; // yaw
#else
        // 否则偏航角仅由陀螺仪积分（注意：长期会漂移）
        float yaw_G = pMpu->gyroZ * Gyro_G;
        if ((yaw_G > 1.0f) || (yaw_G < -1.0f))
        {
            pAngE->yaw += yaw_G * dt;
        }
#endif
        // 俯仰角（pitch）
        pAngE->pitch = asin(2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;
        // 横滚角（roll）
        pAngE->roll  = atan2(2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1,
                             1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;
    }
}