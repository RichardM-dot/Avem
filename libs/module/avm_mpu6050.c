/*
 * ============================================================================
 * 【中文导读】avm_mpu6050.c —— MPU6050 姿态传感器驱动 + 姿态解算
 * ----------------------------------------------------------------------------
 * 本文件做什么：
 *   1. 通过软件 I2C 读写 MPU6050 寄存器（初始化、读六轴原始数据）。
 *   2. IMU_Comput(): 经典的 Mahony 互补滤波姿态解算——把"陀螺仪积分"与
 *      "加速度计重力方向"融合成四元数，再换算出 Pitch/Roll/Yaw 三个欧拉角。
 *      这是整个飞控的数据源头，也是全项目最值得精读的算法。
 *
 * 数据流：
 *   MPU6050_getStructData() 读原始值 → IMU_Comput() 融合解算 → 更新全局
 *   g_Roll/g_Pitch/g_Yaw → avm_pid.c 的外环把 g_Roll 当反馈量。
 *
 * 姿态解算原理速记（读 IMU_Comput 前先看）：
 *   - 陀螺仪：短期准、长期漂移 → 负责高频"积分推算"；
 *   - 加速度计：长期准（指向重力）、短期噪声大 → 负责低频"纠偏"；
 *   - 两者用 Kp/Ki 加权互补 = 互补滤波；用四元数避免万向节锁。
 * ============================================================================
 */

#include <avm_core.h>

static unsigned char avm_mpu_init(void *);

/* MPU 模块注册信息（模块池 [4]）：上电后 MPU6050 需要至少 7ms 启动时间才能通信 */
avm_module_t avm_mpu_module_st = {
    0,
    NULL,
    avm_mpu_init,
    NULL,
    NULL
};

static unsigned char avm_mpu_init(void *arg) {
    // Delay is required after MPU6050 powered up, At least 7ms
    /* 传感器手册要求上电后等待 ≥7ms 再发 I2C 命令，否则寄存器访问无效 */
    delay_ms(7);
    MPU_init();
    return 0;
}


/*
 * 向 MPU6050 单个寄存器写 1 字节。
 * I2C 时序：START → 发器件地址(0xD0=写) → 等ACK → 发寄存器地址 → 等ACK
 *           → 发数据 → 等ACK → STOP
 */
void MPU_Sigle_Write(unsigned char reg_addr, unsigned char reg_data) {
    IIC_Start();
    IIC_Send(MPU6050_ADDR);          /* 器件地址+写位（0xD0 = 0x68<<1|0） */
    if(!IIC_Wait_Ack())
		LED1 = 1;                    /* 无 ACK 视为传感器未接，点亮 LED 指示 */

    IIC_Send(reg_addr);              /* 指定要写的寄存器 */
    IIC_Wait_Ack();

    IIC_Send(reg_data);              /* 写入的数据 */
    IIC_Wait_Ack();

    IIC_Stop();
}

/*
 * 从单个寄存器读 1 字节。
 * 读操作要"先写寄存器地址定指针，再重新 START 转为读方向"——I2C 复合格式。
 */
unsigned char MPU_Sigle_Read(unsigned reg_addr) {
    unsigned char reg_data;
    IIC_Start();
    IIC_Send(MPU6050_ADDR);          /* 器件地址+写位（先定寄存器指针） */
    IIC_Wait_Ack();

    IIC_Send(reg_addr);
    IIC_Wait_Ack();

    IIC_Start();                     /* 重复 START：切换读方向 */
    IIC_Send(MPU6050_ADDR+1);        /* 器件地址+读位（最低位 1 = 读） */
    IIC_Wait_Ack();

    reg_data = IIC_Read();           /* 读 1 字节 */
    IIC_Ack(1);
    IIC_Stop();
    return reg_data;
}

/* 读 16 位数据：寄存器 addr 存高字节、addr+1 存低字节（MPU6050 大端布局） */
short MPU_GetData(unsigned char REG_Addr) {
    unsigned char H, L;
    H = MPU_Sigle_Read(REG_Addr);
    L = MPU_Sigle_Read(REG_Addr+1);
    return (short)(H<<8)+L;          /* 拼成 16 位有符号数（二进制补码） */
}

/*
 * MPU6050 上电初始化（寄存器配置四件套）：
 *   PWR_MGMT_1=0    : 退出睡眠模式（默认上电即睡眠！不写这行传感器不动）
 *   SMPLRT_DIV=7    : 采样分频 1kHz/(1+7)=125Hz 输出
 *   CONFIG=6        : DLPF 低通滤波最重档（5Hz 带宽，输出更平滑）
 *   GYRO_CONFIG=0x18: 陀螺满量程 ±2000°/s（读数除以16.4 得 °/s）
 *   ACCEL_CONFIG=1  : 加速度满量程 ±8g
 * do-while 包裹：传感器没插好时 WHO_AM_I 读不到 0x68，就反复重试直到插上。
 */
void MPU_init() {
    IIC_init();
    do {
        MPU_Sigle_Write(PWR_MGMT_1, 0x00);
        MPU_Sigle_Write(SMPLRT_DIV, 0x07);
        MPU_Sigle_Write(CONFIG, 0x06);
        MPU_Sigle_Write(GYRO_CONFIG, 0x18);
        MPU_Sigle_Write(ACCEL_CONFIG, 0x01);
    } while(IMU_NOT_CONNECTED);      /* WHO_AM_I != 0x68 就一直重试 */
}

/*
 * 读全部六轴并换算成物理单位，填进 SixAxis 结构体。
 * 陀螺：原始值/16.4 = °/s（±2000dps 量程对应灵敏度 16.4 LSB/(°/s)）
 * 加速度：原始值/1671.83 = 单位 g（±8g 量程对应 4096 LSB/g）
 * 后面按编译选项叠加软件零偏校准（静止时读数不为 0，必须扣掉）。
 */
void MPU6050_getStructData(pSixAxis cache) {
    cache->gX = (float)MPU_GetData(GYRO_XOUT_H)/16.4f;
    cache->gY = (float)MPU_GetData(GYRO_YOUT_H)/16.4f;
    cache->gZ = (float)MPU_GetData(GYRO_ZOUT_H)/16.4f;

    cache->aX = (float)MPU_GetData(ACCEL_XOUT_H)/1671.83f;
    cache->aY = (float)MPU_GetData(ACCEL_YOUT_H)/1671.83f;
    cache->aZ = (float)MPU_GetData(ACCEL_ZOUT_H)/1671.83f;

#ifdef IMU_SOFTWARE_FIXED
    /* 软件校零：把实测零偏加回去（数值在 avm_mpu6050.h，静止放置标定所得） */
    cache->gX += G_X_OFFSET;
    cache->gY += G_Y_OFFSET;
    cache->gZ += G_Z_OFFSET;

    cache->aX += A_X_OFFSET;
    cache->aY += A_Y_OFFSET;
    cache->aZ += A_Z_OFFSET;
#endif
}
// void MPU6050_debug(pSixAxis cache) {
//     uart_Float2Char((float)cache->gX);
//     uart_sendData(' ');
//     uart_Float2Char((float)cache->gY);
//     uart_sendData(' ');
//     uart_Float2Char((float)cache->gZ);
//     uart_sendData(' ');
//
//     uart_Float2Char((float)cache->aX);
//     uart_sendData(' ');
//     uart_Float2Char((float)cache->aY);
//     uart_sendData(' ');
//     uart_Float2Char((float)cache->aZ);
//     uart_sendData(' ');
//     uart_sendData(0x0D);
//     uart_sendData(0x0A);
// }

/* 全局姿态输出：三个欧拉角（度）+ 原始六轴数据缓存。
 * g_Roll 被 avm_pid.c 的 Feedback 指针引用——这就是"传感器→控制"的接缝。 */
float g_Yaw, g_Pitch, g_Roll;
SixAxis avm_euler;

/*
 * 姿态解算：Mahony 互补滤波（经典入门实现，也叫"四元数+PI补偿"法）。
 * 输入：cache = 本周期换算好的六轴数据（陀螺 °/s，加速度 g）
 * 输出：更新全局 g_Pitch/g_Roll/g_Yaw（单位：度，57.3 ≈ 180/π）
 *
 * 步骤（与代码逐段对应）：
 *  1) 加速度向量归一化（只取方向，去掉幅值噪声）
 *  2) 用当前四元数推算"理论上重力方向"(vx,vy,vz)
 *  3) 实测重力方向与理论值叉积 = 姿态误差 (ex,ey,ez)
 *  4) 误差经 Ki 积分 + Kp 比例补偿到陀螺仪读数
 *  5) 补偿后的角速度积分更新四元数
 *  6) 四元数归一化（保持单位四元数）
 *  7) 四元数→欧拉角换算
 */
void IMU_Comput(SixAxis cache) {
	static float g_q0 = 1, g_q1 = 0, g_q2 = 0, g_q3 = 0;   //Quaternion  /* 四元数，初始=单位四元数（水平姿态） */
	static float g_exInt = 0, g_eyInt = 0, g_ezInt = 0;    /* PI 补偿的误差积分项（static 跨周期保留） */


    float norm;     //模
    float vx, vy, vz;    /* 由四元数估计的重力方向分量 */
    float ex, ey, ez;    /* 叉积得到的姿态误差分量 */

    norm = sqrt(cache.aX*cache.aX + cache.aY*cache.aY + cache.aZ*cache.aZ);     //取模

    //向量化
    /* 步骤1：加速度归一化——加速度计只信"方向"，幅值受运动加速度污染不可信 */
    cache.aX = cache.aX / norm;
    cache.aY = cache.aY / norm;
    cache.aZ = cache.aZ / norm;

    //估计方向的重力
    /* 步骤2：由当前四元数算出"机体系下的重力分量"（四元数旋转矩阵相关项） */
    vx = 2 * (g_q1 * g_q3 - g_q0 * g_q2);
    vy = 2 * (g_q0 * g_q1 + g_q2 * g_q3);
    vz = g_q0*g_q0 - g_q1*g_q1 - g_q2*g_q2 + g_q3*g_q3;

    //错误的领域和方向传感器测量参考方向几件的交叉乘积的总和
    /* 步骤3：实测重力 × 估计重力 的叉积 = 当前姿态误差向量（哪边歪了、歪多少） */
    ex = (cache.aY * vz - cache.aZ * vy);
    ey = (cache.aZ * vx - cache.aX * vz);
    ez = (cache.aX * vy - cache.aY * vx);

    //积分误差比例积分增益
    /* 步骤4a：误差积分（Ki 项）——慢慢消除陀螺仪的恒定漂移 */
    g_exInt += ex * Ki;
    g_eyInt += ey * Ki;
    g_ezInt += ez * Ki;

    //调整后的陀螺仪测量
    /* 步骤4b：用 Kp*误差 + 积分项 修正陀螺仪读数——"信陀螺，但被加速度计拉着走" */
    cache.gX += Kp * ex + g_exInt;
    cache.gY += Kp * ey + g_eyInt;
    cache.gZ += Kp * ez + g_ezInt;

    //整合四元数率和正常化
    /* 步骤5：四元数微分方程一阶积分（halfT=采样周期一半，见 avm_mpu6050.h）。
     * 物理含义：按修正后的角速度把姿态"推"进一步 */
    g_q0 += (-g_q1 * cache.gX - g_q2 * cache.gY - g_q3 * cache.gZ) * halfT;
    g_q1 += (g_q0 * cache.gX + g_q2 * cache.gZ - g_q3 * cache.gY) * halfT;
    g_q2 += (g_q0 * cache.gY - g_q1 * cache.gZ + g_q3 * cache.gX) * halfT;
    g_q3 += (g_q0 * cache.gZ + g_q1 * cache.gY - g_q2 * cache.gX) * halfT;

    //正常化四元
    /* 步骤6：单位四元数必须保持模长=1（浮点误差会让它漂移），除以模长 */
    norm = sqrt(g_q0*g_q0 + g_q1*g_q1 + g_q2*g_q2 + g_q3*g_q3);
    g_q0 = g_q0 / norm;
    g_q1 = g_q1 / norm;
    g_q2 = g_q2 / norm;
    g_q3 = g_q3 / norm;

    /* 步骤7：四元数 → 欧拉角。
     * Pitch 用 asin（范围±90°），Roll/Yaw 用 atan2（全范围）。
     * 注意：没有磁力计时 Yaw 会缓慢漂移（表现为机头慢慢歪），属正常现象 */
    g_Pitch = asin(-2 * g_q1 * g_q3 + 2 * g_q0 * g_q2) * 57.3;
    g_Roll = atan2(2 * g_q2 * g_q3 + 2 * g_q0 * g_q1, -2 * g_q1*g_q1 - 2 * g_q2*g_q2 + 1) * 57.3;
    g_Yaw = atan2(2 * (g_q1 * g_q2 + g_q0 * g_q3), g_q0*g_q0 + g_q1*g_q1 - g_q2*g_q2 - g_q3*g_q3) * 57.3;
}
