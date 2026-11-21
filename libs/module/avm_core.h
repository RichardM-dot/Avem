#ifndef _AVM_CORE_H_
#define _AVM_CORE_H_
/*
 * ============================================================================
 * 【中文导读】avm_core.h —— 飞控模块体系的核心头文件
 * ----------------------------------------------------------------------------
 * 职责：
 *   1. 定义 avm_module_t（模块接口协议）：全工程每个功能模块（UART/I2C/
 *      Motor/MPU6050）都实现这个结构体，被 avm_core.c 的模块池统一管理——
 *      这是本工程最核心的架构抽象（C 语言版"插件接口"）。
 *   2. 汇总 include 全部模块头文件：其它 .c 只需 include <avm_core.h> 一行
 *      即可看到整个系统（注意循环依赖靠前置 typedef struct 声明解开）。
 *   3. 提供 delay_ms/delay_us 声明与 jump2ISP/中断分组配置。
 *
 * 数据流位置：main() → avm_core_module_st.init_module()（本文件定义的接口）
 *             → 遍历 avm_modules_pool[] 逐个调各模块的 init_module。
 * ============================================================================
 */

/* 前置类型声明：让 avm_module_s 结构体内部能引用自身类型体系，
 * 同时避免与下面各模块头文件形成 #include 循环 */
typedef struct avm_module_s     avm_module_t;
// typedef struct avm_string_s     avm_string_t;

#include <math.h>
#include <stm32f10x.h>
// #include <avm_string.h>
#include <avm_bit.h>
#include <avm_mpu6050.h>
#include <avm_motor.h>
#include <avm_uart.h>
#include <avm_wifi.h>
#include <avm_pid.h>
#include <avm_i2c.h>
#include <avm_tty.h>

/* 【核心抽象】模块接口协议：每个功能模块用这个结构体"自描述"。
 * 类比 C++：init_module ≈ 构造函数，enable/disable ≈ 生命周期钩子（本工程留空）。
 * 各模块在各自 .c 里定义一份实例（如 avm_uart_module_st），再挂进 avm_modules_pool[]。
 * 字段说明：
 *   index          —— 模块在池中的编号，由 avm_core_init() 第一遍遍历时填写
 *   confs          —— 指向该模块配置数组（如 uart 的 {72, 115200}），void* 万能指针
 *   init_module    —— 初始化函数指针，统一签名 unsigned char (*)(void *arg)
 *   enable_module  —— 预留：运行期启用模块（本工程全部为 NULL）
 *   disable_module —— 预留：运行期停用模块（本工程全部为 NULL） */
struct avm_module_s {
    unsigned short  index;
    unsigned int    *confs;
    unsigned char   (*init_module)(void *arg);
    unsigned char   (*enable_module)(void);
    unsigned char   (*disable_module)(void);
};

/* 模块池数组（定义在 avm_core.c，末尾 NULL 哨兵结束遍历）与核心模块实例 */
extern avm_module_t *avm_modules_pool[];
extern avm_module_t avm_core_module_st;

/* SysTick 阻塞延时（实现见 avm_core.c）：t 的单位分别为毫秒/微秒 */
void delay_ms(unsigned int t);
void delay_us(unsigned int t);

/* NVIC 中断优先级分组参数：CMSIS 编码里 0x07-NVIC_GROUPING 即
 * NVIC_SetPriorityGrouping(4)，4 位抢占 + 0 位子优先级（Avem 没有嵌套需求） */
#define NVIC_GROUPING	3
// Jump to built-in bootloader
/* 系统存储器（内置 Bootloader）基地址：出厂固化 ISP 固件，串口烧录用 */
#define ISP_ADDR		0x1FFFF000
void jump2ISP();


#endif