#ifndef _AVM_MPU6050_H_
#define _AVM_MPU6050_H_
/*
 * ============================================================================
 * 【中文导读】avm_mpu6050.h —— MPU6050 寄存器地图 + 姿态解算参数 + 六轴类型
 * ----------------------------------------------------------------------------
 * 职责：
 *   1. MPU6050 寄存器地址表（I2C 读写的"地图"）：上电配置 4 个 + 数据出口 14 个。
 *   2. 软件零偏标定值（IMU_SOFTWARE_FIXED 分支）：陀螺仪静止时读数不为 0 的
 *      残余偏差，实测所得，读取时加回去。
 *   3. Mahony 互补滤波参数 Kp/Ki/halfT 与解算输出的全局欧拉角声明。
 *   4. SixAxis 结构体：全工程六轴数据的统一载体（数据流的"车厢"）。
 *
 * 数据流位置：I2C 读寄存器 → 填 SixAxis.avm_euler → IMU_Comput() 解算
 *             → g_Roll/g_Pitch/g_Yaw → PID 外环反馈（g_Roll）。
 * ============================================================================
 */

//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_
/* 板载 LED，用位带（bit-banding）别名地址直接读写单个引脚：
 * GPIOA_BASE+12 即 ODR 寄存器偏移，PA8=LED0（接在 MPU 模块板上的灯），
 * PD2=LED1（I2C 无 ACK 时点亮，见 avm_mpu6050.c 的 MPU_Sigle_Write） */
#define LED0 BIT_ADDR((GPIOA_BASE+12), 8)
#define LED1 BIT_ADDR((GPIOD_BASE+12), 2)
//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_

void delay(volatile unsigned int count);
//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_

/* MPU6050 的 I2C 器件地址：7 位地址 0x68（AD0 接地时），左移 1 位后
 * 最低位是读/写方向位，故发送值为 0xD0（写）/0xD1（读） */
#define MPU6050_ADDR 0xD0 //    0x68 >> 1

/* ---- 上电配置寄存器（MPU_init() 里逐个写）---- */
#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C

/* ---- 数据寄存器：每个物理量占 2 字节，高字节在前（大端布局）。
 *      读的时候从 H 地址连读两字节（MPU_GetData 里分别读 H 和 H+1）---- */
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

/* 温度输出（本工程未读取，寄存器表里保留完整性） */
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42

#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

/* 电源管理寄存器：上电默认睡眠(0x40)，必须写 0 才开始输出数据；
 * WHO_AM_I 只读：恒返回 0x68，用做"传感器是否在位"的自检探针 */
#define PWR_MGMT_1      0x6B
#define WHO_AM_I        0x75

//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_
/* 启用软件零偏补偿（MPU6050_getStructData 里把下面对应宏加回读数）：
 * 数值来自"静置上电、记录均值"的标定，每颗传感器都不同，换芯片要重标 */
#define IMU_SOFTWARE_FIXED

/* 陀螺零偏（单位 °/s）：Roll 轴 1.59、Pitch 轴 0.79、Yaw 轴 2.01——
 * Yaw 零偏最大很典型（Z 轴对震动/温漂最敏感） */
#define G_X_OFFSET 1.5853f
#define G_Y_OFFSET 0.7926f
#define G_Z_OFFSET 2.0121f

/* 加速度零偏（单位 g）：本机 X/Y 标定为 0，Z 轴 0.2g 是静置时的重力读数残差 */
#define A_X_OFFSET 0
#define A_Y_OFFSET 0
#define A_Z_OFFSET 0.2f

/* 在位自检：WHO_AM_I 应返回 0x68；IMU_NOT_CONNECTED 为真则 MPU_init 反复重试 */
#define IMU_ADDRESS 0x68
#define IMU_NOT_CONNECTED (MPU_Sigle_Read(WHO_AM_I)!=IMU_ADDRESS)

extern avm_module_t avm_mpu_module_st;

/* 六轴数据统一载体（数据流的"车厢"）：gX/gY/gZ = 三轴角速度（°/s，已除以
 * 16.4 灵敏度），aX/aY/aZ = 三轴加速度（除以 1671.83 ≈ 16384/9.8，即按 ±2g
 * 量程换算成 m/s² 的系数——与 ACCEL_CONFIG 实际量程存在口径差，但因为
 * IMU_Comput 会把加速度向量归一化，绝对刻度不影响姿态解算结果）。
 * pSixAxis 为其指针类型，函数参数统一用它传递 */
typedef struct{
    float gX;
    float gY;
    float gZ;
    float aX;
    float aY;
    float aZ;
}SixAxis, *pSixAxis;

/* ---- Mahony 互补滤波参数（调姿态"手感"的三个旋钮，见 Docs/04）----
 * Kp：加速度计纠偏力度——太大跟震动走（高频抖），太小漂移收敛慢
 * Ki：误差积分增益——负责消除陀螺恒定零偏（Kp 已兜掉大部分）
 * halfT：采样周期一半。0.001 = 假设 2ms 解算一次（500Hz）；实际调用频率
 * 由控制流决定，与假设失配时四元数积分速度会成比例偏差；
 * 注意四元数微分方程里用的是"修正后角速度"，积分一步推半个周期 */
#define Kp      100.0f      //比例增益支配率(常量)
#define Ki      0.002f      //积分增益支配率
#define halfT   0.001f      //采样周期的一半

/* 姿态解算输出（度）：IMU_Comput() 每周期覆盖写入；
 * g_Roll 被 avm_pid 的 Feedback 指针绑定——姿态到控制的"接线柱" */
extern float g_Yaw, g_Pitch, g_Roll;
extern SixAxis avm_euler;

/* ---- 驱动函数声明（实现与逐行注释见 avm_mpu6050.c）----
 * MPU_Sigle_Write/Read ：寄存器级单字节写/读（复合格式 I2C 时序）
 * MPU_GetData          ：读 16 位有符号原始值（H<<8|L）
 * MPU_init             ：上电配置四件套 + WHO_AM_I 在位重试
 * MPU6050_getStructData：读六轴并换算成物理单位（含软件零偏补偿）
 * IMU_Comput           ：Mahony 四元数姿态解算（本工程核心算法） */
void MPU_Sigle_Write(unsigned char reg_addr, unsigned char reg_data);
unsigned char MPU_Sigle_Read(unsigned reg_addr);
short MPU_GetData(unsigned char REG_Addr);
void MPU_init();
void MPU6050_getStructData(pSixAxis cache);
// void MPU6050_debug(pSixAxis cache);
void IMU_Comput(SixAxis);

#endif
