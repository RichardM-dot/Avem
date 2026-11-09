/*
 * ============================================================================
 * 【中文导读】avm_core.c —— Avem 飞控的核心模块
 * ----------------------------------------------------------------------------
 * 本文件做什么：
 *   1. 实现"模块池"机制：把 UART/I2C/电机/MPU6050 各模块的初始化函数挂进一张
 *      函数表 avm_modules_pool[]，core 模块作为第 0 号，启动时统一遍历初始化。
 *      这是一种轻量的"注册-初始化"模式，C 语言里模拟"构造函数"的常见写法。
 *   2. 提供基于 SysTick 定时器的毫秒/微秒级阻塞延时（裸机最常用的延时方案）。
 *   3. 提供 jump2ISP()：跳转到芯片内置 Bootloader（串口 ISP 烧录模式）。
 *
 * 在整体架构中的位置：
 *   main() 只调用 avm_core_module_st.init_module()，由本文件按池顺序依次把
 *   UART→I2C→Motor→MPU 全部初始化好，然后进入空转主循环（真正的控制在定时器
 *   中断里跑，见 startup.c 的向量表与 motor.c 的 TIM3）。
 *
 * 关键函数速览：
 *   - avm_core_init():    遍历模块池，给每个模块编号并调用其 init_module
 *   - delay_ms/delay_us:  SysTick 倒计时阻塞延时（72MHz 主频下 9000 tick = 1ms）
 *   - jump2ISP():         关闭 SysTick → 重设主栈指针 MSP → 跳到系统 Bootloader
 * ============================================================================
 */

#include <avm_core.h>

static unsigned char avm_core_init(void *arg);

/* 核心模块自身也遵循 avm_module_t 接口（自己管理自己，index=0）。
 * 结构体字段顺序见 avm_core.h：{index, confs, init_module, enable, disable} */
avm_module_t avm_core_module_st = {
	0,
	NULL,
	avm_core_init,
	NULL,
	NULL
};

/*
 * 跳转到 STM32 内置系统 Bootloader（ISP 串口烧录区）。
 * 原理：芯片出厂在 0x1FFFF000 固化了 Bootloader，正常启动流程不会进入它；
 * 这里手动完成"假装从系统存储器启动"的三步：
 */
void jump2ISP() {
	SysTick->CTRL = 0;                                   /* 1. 关闭 SysTick，防止跳转途中触发中断 */
	SysTick->VAL = 0;                                    /*    清空当前计数值 */
	__set_MSP(*(unsigned int *)ISP_ADDR);                /* 2. 把主栈指针 MSP 设为 Bootloader 区首 4 字节存的栈顶值 */
	((void (*)(void))*((unsigned int *)(ISP_ADDR + 4)))(); /* 3. 取该处第 2 个字（复位向量）并跳转执行 */
}

/* 毫秒级阻塞延时。
 * 9000 = 72MHz / 8 / 1000：SysTick 使用 HCLK/8=9MHz 时钟，9000 个计数正好 1ms */
void delay_ms(unsigned int t) {
	SysTick->LOAD = 9000 * t;      /* 装入倒计数值 */
	SysTick->VAL = 0;              /* 清零当前值（写任意值都会清 COUNTFLAG） */
	SysTick->CTRL = 0x01;          /* 使能 SysTick，时钟源 = HCLK/8（bit2=0），开始倒数 */
	/* 自旋等待：bit0=ENABLE 仍为 1 且 bit16(COUNTFLAG) 置位（计到 0）时退出 */
	for(unsigned int tmp = SysTick->CTRL;(tmp&0x01)&&(!(tmp&SysTick_CTRL_COUNTFLAG));tmp = SysTick->CTRL);
	SysTick->CTRL = 0;             /* 关闭 SysTick */
	SysTick->VAL = 0;              /* 清计数值 */
}

// 延时函数, 单位为微秒
// 原理同 delay_ms：9MHz 时钟下 9 个计数 = 1µs
void delay_us(unsigned int t) {
	SysTick->LOAD = 9 * t;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x01;
	for(unsigned int tmp = SysTick->CTRL;(tmp&0x01)&&(!(tmp&SysTick_CTRL_COUNTFLAG));tmp = SysTick->CTRL);
	SysTick->CTRL = 0;
	SysTick->VAL = 0;
}

/* 模块池：飞控全部功能模块的初始化入口集中在此（工厂目录）。
 * 顺序即初始化顺序：先 UART（调试口）→ I2C（MPU 依赖它）→ 电机 → MPU6050。
 * 末尾 NULL 作为遍历结束哨兵。各 xxx_module_st 定义在各自 .c 文件里。 */
avm_module_t *avm_modules_pool[] = {
	&avm_core_module_st,       /* [0] 核心（本文件，自管理） */
    &avm_uart_module_st,       /* [1] 串口（MultiWii 协议/调试输出） */
	&avm_i2c_module_st,        /* [2] 软件 I2C 总线 */
	&avm_motor_module_st,      /* [3] 四路电机 PWM（TIM3） */
	&avm_mpu_module_st,        /* [4] MPU6050 姿态传感器 */
    NULL
};

/* 核心初始化：由 main() 显式调用。
 * 它先给池中每个模块发编号，再从 [1] 开始逐个调用各模块自己的 init_module，
 * 完成"一张表启动整个系统"。 */
static unsigned char avm_core_init(void *arg) {
	/* 设置 NVIC 中断优先级分组。0x07 是全部位都归抢占优先级；
	 * 减去 NVIC_GROUPING(3) 即留 3 位给子优先级（这是 CMSIS 函数的编码方式） */
	NVIC_SetPriorityGrouping(0x07 - NVIC_GROUPING);

	avm_module_t *p_core = avm_modules_pool[0];
	// Count
	/* 第一遍遍历：统计模块个数并给每个模块填 index 编号 */
	static unsigned char avm_modules_num = 0;
	for(;avm_modules_pool[avm_modules_num] != NULL;avm_modules_num++) {
		avm_modules_pool[avm_modules_num]->index = avm_modules_num;
	}

	p_core->confs = (void *)&avm_modules_num;	// Stored
	// Initial
	/* 第二遍遍历：从 [1] 开始（跳过 core 自己），逐个调用各模块的初始化函数。
	 * 注意变量名 this 只是普通指针名（C 里没有 C++ 的 this）。 */
	avm_module_t **this = &avm_modules_pool[1];
	for(;*this != NULL;this++) {
			(*this)->init_module((*this)->confs);
	}

	return 0;
}
