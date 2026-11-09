/*
 * ============================================================================
 * 【中文导读】avm_i2c.c —— 软件 I2C（GPIO 模拟时序）
 * ----------------------------------------------------------------------------
 * 本文件做什么：
 *   不用硬件 I2C 外设，用两个普通 GPIO（PB13=SCL、PB15=SDA）按 I2C 协议
 *   时序逐位翻转电平，实现 Start/Stop/发字节/收字节/ACK 五个原语。
 *   优点：任意引脚可用、时序完全可控、调试直观——学习 I2C 协议的最佳教材。
 *
 * I2C 协议速记（读下面的时序函数前先看）：
 *   - 空闲态：SDA、SCL 都为高（上拉）。
 *   - Start：SCL 高时 SDA 由高→低；Stop：SCL 高时 SDA 由低→高。
 *   - 数据有效性：SCL 高电平期间 SDA 必须稳定，SCL 低电平期间才允许变化。
 *   - 每传 8 位，接收方在第 9 个时钟回 ACK（拉低 SDA）或 NAK（保持高）。
 * ============================================================================
 */

#include <avm_core.h>

static unsigned char avm_i2c_init(void *arg);

/* I2C 模块注册信息（模块池 [2]），init 时配置 GPIO 并释放总线到空闲态 */
avm_module_t avm_i2c_module_st = {
    0,
    NULL,
    avm_i2c_init,
    NULL,
    NULL
};

unsigned char avm_i2c_init(void *arg) {
    IIC_init();
    return 0;
}

/* 简单软件延时（nus≈微秒量级，4 次空转约 1µs @72MHz），I2C 速率的节拍来源 */
static void delay_custom(volatile unsigned int nus) {
    for(nus *= 4; nus; nus--);
}

/* I2C 总线初始化：开 GPIOB 时钟，PB13/PB15 配为输出模式，释放总线为空闲态 */
void IIC_init() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;       //GPIOB enable
    GPIOB->CRH &= 0x00FFFFFF;   //reset B13 & B15
    GPIOB->CRH |= 0x77000000;   //Set B13 & B15 b0111
    SDA = 1;
    SCL = 1;

//这里B14和B15设置为开漏输出模式(01), 其实一般都用推挽输出模式。
//开漏输出模式有个"好的Bug": 在输出状态下，输入信号没有关断，后面的路也是畅通的
//最终就会进入输入寄存器，导致在输出状态下，输入功能保持正常。
//如果用推挽输出的话，IIC通讯时每次读取SDA之前都要配置SDA为输入，才能读取IDR

//    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;       //GPIOC enable
//    GPIOC->CRL &= 0xF0FFFFFF;
//    GPIOC->CRL |= 0x03000000;   //推挽输出, AD0 ----> PC6
//    AD0 = 0;
}

/* 起始信号：SCL 高电平期间 SDA 产生下降沿 */
void IIC_Start() {
    SDA = 1;      /* 先确保两线都为高（空闲态起点） */
    SCL = 1;
    IIC_DELAY();
    SDA = 0;      /* SDA 先拉低 —— Start 的标志动作 */
    IIC_DELAY();
    SCL = 0;      /* SCL 再拉低，总线进入"数据传输预备"状态 */
}

/* 停止信号：SCL 高电平期间 SDA 产生上升沿（与 Start 相反） */
void IIC_Stop() {
    SCL = 0;
    IIC_DELAY();
    SDA = 0;      /* 先把 SDA 拉低，制造"低→高"的条件 */
    IIC_DELAY();
    SCL = 1;      /* 抬高 SCL */
    IIC_DELAY();
    SDA = 1;      /* SCL 高时 SDA 上升沿 = Stop */
    IIC_DELAY();
}

/* 主机发送 ACK/NAK：ack=0 拉低 SDA 表示应答，ack=1 保持高表示非应答（读结束） */
void IIC_Ack(unsigned char ack) {
    SCL = 0;      /* SCL 低电平期间才允许改 SDA */
    IIC_DELAY();
    SDA = ack;    /* 放好应答电平 */
    IIC_DELAY();
    SCL = 1;      /* 第 9 个时钟上升沿，从机在这里采样 */
    IIC_DELAY();
    SCL = 0;
    IIC_DELAY();
}

/* 等待从机应答：第 9 个时钟释放 SDA 读回电平。
 * 返回 1 = 收到 ACK（从机拉低了 SDA）；返回 0 = 无应答（可能器件不存在/故障）。
 * 注意本文件里调用方用 if(!IIC_Wait_Ack()) 判"无应答报错"。 */
unsigned char IIC_Wait_Ack() {
    SCL = 0;
    IIC_DELAY();
    SDA = 1;      /* 释放 SDA（交给从机控制） */
    IIC_DELAY();
    SCL = 1;      /* 第 9 个时钟 */
    IIC_DELAY();
    if(READ_SDA) {         /* SDA 为高 = 从机没有拉低 = 无 ACK */
        SCL = 0;
        IIC_DELAY();
        return 0;
    }

    SCL = 0;      /* 收到 ACK，拉低 SCL 结束第 9 个时钟 */
    return 1;
}

/* 发送 1 字节：高位在前（MSB first），每个位在 SCL 低电平时放到 SDA 上，
 * SCL 拉高让从机采样，再拉低准备下一位 */
void IIC_Send(unsigned char dat) {
    unsigned char i;
    SCL = 0;
    for(i = 0; i < 8; i++) {
        IIC_DELAY();
        SDA = (dat&0x80)>>7;   /* 取最高位放到 SDA */
        dat <<= 1;             /* 左移，准备下一位 */
        IIC_DELAY();
        SCL = 1;               /* SCL 高：从机采样当前位 */
        IIC_DELAY();
        SCL = 0;               /* SCL 低：允许换下一位 */
    }
}

/* 接收 1 字节：同样 MSB first，在 SCL 高电平期间采样 SDA，
 * 每位左移进 dat 的最低位 */
unsigned char IIC_Read() {
    unsigned char i, dat = 0;
    SDA = 1;      /* 释放 SDA，由从机驱动数据 */
    for(i = 0; i < 8; i++) {
        SCL = 0;
        IIC_DELAY();
        SCL = 1;               /* SCL 高：读一个位 */
        IIC_DELAY();
        dat<<=1;
        dat |= READ_SDA;       /* 拼进最低位 */
    }
    SCL = 0;
    return dat;
}
