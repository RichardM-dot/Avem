/*
 * ============================================================================
 * 【中文导读】startup.c —— 上电启动代码（用 C 语言写的启动文件）
 * ----------------------------------------------------------------------------
 * 裸机程序从复位到进入 main() 之前发生的事，全部在本文件完成：
 *   1. 硬件复位后 Cortex-M3 自动从向量表[0]取出初始 MSP、从向量表[1]取出
 *      Reset_Handler 地址并跳转执行（见下方 g_pfnVectors[]）。
 *   2. Reset_Handler 先调 __Init_Data()：把 .data 段从 Flash 拷到 RAM、
 *      把 .bss 段清零——这就是"全局变量为什么不用手动初始化"的底层原因。
 *   3. 设置 SCB->VTOR 向量表偏移、调 SystemInit() 配置时钟，最后进入 main()。
 *
 * 与常见工程的差异：多数模板用汇编 startup_stm32f10x_hd.s，本工程改用纯 C
 * 实现（结构体数组当向量表），可读性更好。链接符号 _sidata/_sdata/_edata/
 * _sbss/_ebss/_estack 由 linker.ld 提供（.data 段 ">RAM AT>FLASH" 的 LMA/VMA
 * 双地址布局正是为这份拷贝代码服务的）。
 *
 * 注：本文件为 C 语言启动文件，按 A 级注释"关键段"：向量表结构、
 *     __Init_Data、Reset_Handler 三处；weak 声明区仅加导读不逐行展开。
 * ============================================================================
 */
#include "stm32f10x.h"
typedef void( *const intfunc )( void );
#define WEAK __attribute__ ((weak))
/* provided by the linker script */
//extern unsigned long _etext; /* start address of the static initialization data */
/* 链接脚本（linker.ld）导出的 6 个关键地址符号，__Init_Data()/Reset_Handler 全靠它们：
 *   _sidata = .data 段在 Flash 中的存放地址（LMA，装载地址）
 *   _sdata/_edata  = .data 段在 RAM 中的起止地址（VMA，运行地址）
 *   _sbss/_ebss    = .bss 段在 RAM 中的起止地址（需清零）
 *   _estack        = RAM 末尾 = 初始主栈指针 MSP（64K RAM 时即 0x20010000） */
extern unsigned long _sidata; /* start address of the static initialization data */
extern unsigned long _sdata; /* start address of the data section */
extern unsigned long _edata; /* end address of the data section */
extern unsigned long _sbss; /* start address of the bss section */
extern unsigned long _ebss; /* end address of the bss section */
extern unsigned long _estack; /* end address of the stack section */

/* __interrupt__ 属性：告诉编译器这是中断入口，生成相应的返回语义（bx lr 前恢复现场） */
void Reset_Handler(void) __attribute__((__interrupt__));
void __Init_Data(void);
void Default_Handler(void);

extern int main(void);

void WEAK NMI_Handler(void);
void WEAK HardFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK BusFault_Handler(void);
void WEAK UsageFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK SVC_Handler(void);
void WEAK DebugMon_Handler(void);
void WEAK PendSV_Handler(void);
void WEAK SysTick_Handler(void);
void WEAK WWDG_IRQHandler(void);
void WEAK PVD_IRQHandler(void);
void WEAK TAMPER_IRQHandler(void);
void WEAK RTC_IRQHandler(void);
void WEAK FLASH_IRQHandler(void);
void WEAK RCC_IRQHandler(void);
void WEAK EXTI0_IRQHandler(void);
void WEAK EXTI1_IRQHandler(void);
void WEAK EXTI2_IRQHandler(void);
void WEAK EXTI3_IRQHandler(void);
void WEAK EXTI4_IRQHandler(void);
void WEAK DMA1_Channel1_IRQHandler(void);
void WEAK DMA1_Channel2_IRQHandler(void);
void WEAK DMA1_Channel3_IRQHandler(void);
void WEAK DMA1_Channel4_IRQHandler(void);
void WEAK DMA1_Channel5_IRQHandler(void);
void WEAK DMA1_Channel6_IRQHandler(void);
void WEAK DMA1_Channel7_IRQHandler(void);
void WEAK ADC1_2_IRQHandler(void);
void WEAK USB_HP_CAN1_TX_IRQHandler(void);
void WEAK USB_LP_CAN1_RX0_IRQHandler(void);
void WEAK CAN1_RX1_IRQHandler(void);
void WEAK CAN1_SCE_IRQHandler(void);
void WEAK EXTI9_5_IRQHandler(void);
void WEAK TIM1_BRK_IRQHandler(void);
void WEAK TIM1_UP_IRQHandler(void);
void WEAK TIM1_TRG_COM_IRQHandler(void);
void WEAK TIM1_CC_IRQHandler(void);
void WEAK TIM2_IRQHandler(void);
void WEAK TIM3_IRQHandler(void);
void WEAK TIM4_IRQHandler(void);
void WEAK I2C1_EV_IRQHandler(void);
void WEAK I2C1_ER_IRQHandler(void);
void WEAK I2C2_EV_IRQHandler(void);
void WEAK I2C2_ER_IRQHandler(void);
void WEAK SPI1_IRQHandler(void);
void WEAK SPI2_IRQHandler(void);
void WEAK USART1_IRQHandler(void);
void WEAK USART2_IRQHandler(void);
void WEAK USART3_IRQHandler(void);
void WEAK EXTI15_10_IRQHandler(void);
void WEAK RTCAlarm_IRQHandler(void);
void WEAK USBWakeUp_IRQHandler(void);
void WEAK TIM8_BRK_IRQHandler(void);
void WEAK TIM8_UP_IRQHandler(void);
void WEAK TIM8_TRG_COM_IRQHandler(void);
void WEAK TIM8_CC_IRQHandler(void);
void WEAK ADC3_IRQHandler(void);
void WEAK FSMC_IRQHandler(void);
void WEAK SDIO_IRQHandler(void);
void WEAK TIM5_IRQHandler(void);
void WEAK SPI3_IRQHandler(void);
void WEAK UART4_IRQHandler(void);
void WEAK UART5_IRQHandler(void);
void WEAK TIM6_IRQHandler(void);
void WEAK TIM7_IRQHandler(void);
void WEAK DMA2_Channel1_IRQHandler(void);
void WEAK DMA2_Channel2_IRQHandler(void);
void WEAK DMA2_Channel3_IRQHandler(void);
void WEAK DMA2_Channel4_5_IRQHandler(void);

/*
 * 【关键段 1】中断向量表（Cortex-M3 复位后硬件自动加载的"函数指针数组"）
 * ----------------------------------------------------------------------------
 * 布局规则（固定，由 ARMv7-M 架构定义，不是 STM32 发明的）：
 *   表[0]  = 初始 MSP 栈顶值（不是函数！复位后硬件自动弹给 SP 寄存器）
 *   表[1]  = Reset_Handler        —— 复位入口，唯一必转的入口
 *   表[2..15] = 内核异常（NMI/HardFault/MemManage/BusFault/UsageFault/
 *               SVC/DebugMon/PendSV/SysTick，中间夹保留项）
 *   表[16..]  = 外设中断 IRQ0~IRQ67（WWDG 起到 DMA2_Channel4_5 止，
 *               顺序严格对应《STM32F10x 参考手册》中断映射表，不可乱序）
 *
 * 放在 .isr_vectors 段：linker.ld 用 KEEP(*(.isr_vectors)) 把它钉在 Flash
 * 起始 0x08000000 处（并断言 0x80 对齐——VTOR 要求向量表至少 128 字对齐）。
 * weak 声明 + 文件尾 #pragma weak 别名的配合：工程里没实现的中断处理函数
 * 全部自动落到 Default_Handler() 死循环，防止误触发中断时跑飞。
 */
__attribute__ ((section(".isr_vectors")))
void (* const g_pfnVectors[])(void) = {
 (intfunc)((unsigned long)&_estack), /* The stack pointer after relocation */
 Reset_Handler,
/* Reset Handler */
 NMI_Handler,
/* NMI Handler */
 HardFault_Handler,
/* Hard Fault Handler */
 MemManage_Handler,
/* MPU Fault Handler */
 BusFault_Handler,
/* Bus Fault Handler */
 UsageFault_Handler,
/* Usage Fault Handler */
 0,
/* Reserved */
 0,
/* Reserved */
 0,
/* Reserved */
 0,
/* Reserved */
 SVC_Handler,
/* SVCall Handler */
 DebugMon_Handler,
/* Debug Monitor Handler */
 0,
/* Reserved */
 PendSV_Handler,
/* PendSV Handler */
 SysTick_Handler,
/* SysTick Handler */
/* External Interrupts */
 WWDG_IRQHandler,
/* Window Watchdog */
 PVD_IRQHandler,
/* PVD through EXTI Line detect */
 TAMPER_IRQHandler,
/* Tamper */
 RTC_IRQHandler,
/* RTC */
 FLASH_IRQHandler,
/* Flash */
 RCC_IRQHandler,
/* RCC */
 EXTI0_IRQHandler,
/* EXTI Line 0 */
 EXTI1_IRQHandler,
/* EXTI Line 1 */
 EXTI2_IRQHandler,
/* EXTI Line 2 */
 EXTI3_IRQHandler,
/* EXTI Line 3 */
 EXTI4_IRQHandler,
/* EXTI Line 4 */
 DMA1_Channel1_IRQHandler, /* DMA1 Channel 1 */
 DMA1_Channel2_IRQHandler, /* DMA1 Channel 2 */
 DMA1_Channel3_IRQHandler, /* DMA1 Channel 3 */
 DMA1_Channel4_IRQHandler, /* DMA1 Channel 4 */
 DMA1_Channel5_IRQHandler, /* DMA1 Channel 5 */
 DMA1_Channel6_IRQHandler, /* DMA1 Channel 6 */
 DMA1_Channel7_IRQHandler, /* DMA1 Channel 7 */
 ADC1_2_IRQHandler,
/* ADC1 & ADC2 */
 USB_HP_CAN1_TX_IRQHandler, /* USB High Priority or CAN1 TX */
 USB_LP_CAN1_RX0_IRQHandler, /* USB Low Priority or CAN1 RX0 */
 CAN1_RX1_IRQHandler,
/* CAN1 RX1 */
 CAN1_SCE_IRQHandler,
/* CAN1 SCE */
 EXTI9_5_IRQHandler,
/* EXTI Line 9..5 */
 TIM1_BRK_IRQHandler,
/* TIM1 Break */
 TIM1_UP_IRQHandler,
/* TIM1 Update */
 TIM1_TRG_COM_IRQHandler, /* TIM1 Trigger and Commutation */
 TIM1_CC_IRQHandler,
/* TIM1 Capture Compare */
 TIM2_IRQHandler,
/* TIM2 */
 TIM3_IRQHandler,
/* TIM3 */
 TIM4_IRQHandler,
/* TIM4 */
 I2C1_EV_IRQHandler,
/* I2C1 Event */
 I2C1_ER_IRQHandler,
/* I2C1 Error */
 I2C2_EV_IRQHandler,
/* I2C2 Event */
 I2C2_ER_IRQHandler,
/* I2C2 Error */
 SPI1_IRQHandler,
/* SPI1 */
 SPI2_IRQHandler,
/* SPI2 */
 USART1_IRQHandler,
/* USART1 */
 USART2_IRQHandler,
/* USART2 */
 USART3_IRQHandler,
/* USART3 */
 EXTI15_10_IRQHandler,
/* EXTI Line 15..10 */
 RTCAlarm_IRQHandler,
/* RTC Alarm through EXTI Line */
 USBWakeUp_IRQHandler,
/* USB Wakeup from suspend */
 TIM8_BRK_IRQHandler,
 TIM8_UP_IRQHandler,
 TIM8_TRG_COM_IRQHandler,
 TIM8_CC_IRQHandler,
 ADC3_IRQHandler,
 FSMC_IRQHandler,
 SDIO_IRQHandler,
 TIM5_IRQHandler,
 SPI3_IRQHandler,
 UART4_IRQHandler,
 UART5_IRQHandler,
 TIM6_IRQHandler,
 TIM7_IRQHandler,
 DMA2_Channel1_IRQHandler,
 DMA2_Channel2_IRQHandler,
 DMA2_Channel3_IRQHandler,
 DMA2_Channel4_5_IRQHandler,
 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0,
 0, 0, 0,
 (intfunc)0xF1E0F85F
/* @0x1E0. This is for boot in RAM mode for STM32F10x High Density devices. */
};
/*
 * 【关键段 2】__Init_Data：C 运行环境的内存初始化（main() 之前必做）
 * ----------------------------------------------------------------------------
 * 做两件事，对应链接脚本里 .data 的 ">RAM AT>FLASH" 双地址布局：
 *   1) 拷贝 .data：把存在 Flash(_sidata) 里的已初始化全局变量初值，
 *      搬到它们运行时该在的 RAM 地址(_sdata.._edata)。
 *      ——例如 `unsigned int x = 42;` 中那个 42 就躺在这段 Flash 里。
 *   2) 清零 .bss：未显式初始化的全局/静态变量（_sbss.._ebss）全部写 0，
 *      ——这是 C 标准承诺"静态存储期变量默认为 0"的物理实现。
 * 两者都是逐字(unsigned long)搬运，地址天然 4 字节对齐，效率最高。
 */
void __Init_Data(void) {
	unsigned long *src, *dst;
	/* copy the data segment into ram */
	src = &_sidata;
	dst = &_sdata;
	/* src==dst 说明链接器把 .data 直接放进了 RAM（无需搬运），直接跳过 */
	if (src != dst)
		while(dst < &_edata)
			*(dst++) = *(src++);
	/* zero the bss segment */
	dst = &_sbss;
	while(dst < &_ebss)
		*(dst++) = 0;
}
/*
 * 【关键段 3】Reset_Handler：复位入口（向量表[1]指向这里）
 * ----------------------------------------------------------------------------
 * 完整启动时序：硬件选启动区(0x08000000) → 取[0]设 MSP、取[1]跳到此处
 *   → __Init_Data() 备好内存 → 设 VTOR → SystemInit() 拉到 72MHz → main()。
 * 注意：MSP 是硬件自动装载的，软件只负责内存、向量表和时钟三件事。
 */
void Reset_Handler(void) {
	__Init_Data(); /* Initialize memory, data and bss */
	extern u32 _isr_vectors_offs; /* the offset to the vector table in ram */
	/* 设置向量表基址：VTOR 低 7 位恒为 0（要求 128 字对齐），
	 * 0x1FFFFF80 掩码即保留对齐位、清掉其它位。本工程向量表在 Flash，
	 * _isr_vectors_offs=0，结果即 0x08000000 */
	SCB->VTOR = 0x08000000 | ((u32)&_isr_vectors_offs & (u32)0x1FFFFF80); /* set interrupt vector table address */
	SystemInit(); /* configure the clock */
	main(); /* start execution of the program */
	/* main 返回即死等复位——裸机程序没有"退出"的概念 */
	while(1) {}
}
/* 【weak 别名区】把所有"工程未实现"的中断处理函数全部弱别名到 Default_Handler。
 * 机制：#pragma weak X = Y 生成弱符号 X（地址=Y）；一旦别处有强定义 X，别名自动失效。
 * 效果：误触发未处理中断时进 Default_Handler 死循环，而不是跳到随机地址跑飞。
 * 注意：第一行写的是 MMI_Handler——疑似 NMI_Handler 的笔误，NMI 因此没有别名
 * （弱声明未定义，向量表里其表项为 0）。此处仅记录现象，不改代码。 */
#pragma weak MMI_Handler	= Default_Handler
#pragma weak MemManage_Handler	= Default_Handler
#pragma weak BusFault_Handler	= Default_Handler
#pragma weak UsageFault_Handler	= Default_Handler
#pragma weak SVC_Handler	= Default_Handler
#pragma weak DebugMon_Handler	= Default_Handler
#pragma weak PendSV_Handler	= Default_Handler
#pragma weak SysTick_Handler	= Default_Handler
#pragma weak WWDG_IRQHandler	= Default_Handler
#pragma weak PVD_IRQHandler	= Default_Handler
#pragma weak TAMPER_IRQHandler	= Default_Handler
#pragma weak RTC_IRQHandler	= Default_Handler
#pragma weak FLASH_IRQHandler	= Default_Handler
#pragma weak RCC_IRQHandler	= Default_Handler
#pragma weak EXTI0_IRQHandler	= Default_Handler
#pragma weak EXTI1_IRQHandler	= Default_Handler

#pragma weak EXTI2_IRQHandler 	= Default_Handler
#pragma weak EXTI3_IRQHandler	= Default_Handler
#pragma weak EXTI4_IRQHandler	= Default_Handler
#pragma weak DMA1_Channel1_IRQHandler	= Default_Handler
#pragma weak DMA1_Channel2_IRQHandler	= Default_Handler
#pragma weak DMA1_Channel3_IRQHandler	= Default_Handler
#pragma weak DMA1_Channel4_IRQHandler	= Default_Handler
#pragma weak DMA1_Channel5_IRQHandler	= Default_Handler
#pragma weak DMA1_Channel6_IRQHandler	= Default_Handler
#pragma weak DMA1_Channel7_IRQHandler	= Default_Handler
#pragma weak ADC1_2_IRQHandler		= Default_Handler
#pragma weak USB_HP_CAN1_TX_IRQHandler	= Default_Handler
#pragma weak USB_LP_CAN1_RX0_IRQHandler	= Default_Handler
#pragma weak CAN1_RX1_IRQHandler 	= Default_Handler
#pragma weak CAN1_SCE_IRQHandler 	= Default_Handler
#pragma weak EXTI9_5_IRQHandler		= Default_Handler
#pragma weak TIM1_BRK_IRQHandler 	= Default_Handler
#pragma weak TIM1_UP_IRQHandler		= Default_Handler
#pragma weak TIM1_TRG_COM_IRQHandler	= Default_Handler
#pragma weak TIM1_CC_IRQHandler		= Default_Handler
#pragma weak TIM2_IRQHandler		= Default_Handler
#pragma weak TIM3_IRQHandler		= Default_Handler
#pragma weak TIM4_IRQHandler		= Default_Handler
#pragma weak I2C1_EV_IRQHandler		= Default_Handler
#pragma weak I2C1_ER_IRQHandler		= Default_Handler
#pragma weak I2C2_EV_IRQHandler		= Default_Handler
#pragma weak I2C2_ER_IRQHandler		= Default_Handler
#pragma weak SPI1_IRQHandler		= Default_Handler
#pragma weak SPI2_IRQHandler		= Default_Handler
#pragma weak USART1_IRQHandler		= Default_Handler
#pragma weak USART2_IRQHandler		= Default_Handler
#pragma weak USART3_IRQHandler		= Default_Handler
#pragma weak EXTI15_10_IRQHandler	= Default_Handler
#pragma weak RTCAlarm_IRQHandler	= Default_Handler
#pragma weak USBWakeUp_IRQHandler 	= Default_Handler
#pragma weak TIM8_BRK_IRQHandler 	= Default_Handler
#pragma weak TIM8_UP_IRQHandler		= Default_Handler
#pragma weak TIM8_TRG_COM_IRQHandler	= Default_Handler
#pragma weak TIM8_CC_IRQHandler		= Default_Handler
#pragma weak ADC3_IRQHandler		= Default_Handler
#pragma weak FSMC_IRQHandler		= Default_Handler
#pragma weak SDIO_IRQHandler		= Default_Handler
#pragma weak TIM5_IRQHandler		= Default_Handler
#pragma weak SPI3_IRQHandler		= Default_Handler
#pragma weak UART4_IRQHandler		= Default_Handler
#pragma weak UART5_IRQHandler		= Default_Handler
#pragma weak TIM6_IRQHandler		= Default_Handler
#pragma weak TIM7_IRQHandler		= Default_Handler
#pragma weak DMA2_Channel1_IRQHandler	= Default_Handler
#pragma weak DMA2_Channel2_IRQHandler	= Default_Handler
#pragma weak DMA2_Channel3_IRQHandler	= Default_Handler
#pragma weak DMA2_Channel4_5_IRQHandler	= Default_Handler

/* 所有未实现中断的统一兜底：原地死循环。调试时若程序卡死在此，
 * 说明有中断被触发却没有对应的处理函数（用调试器看 LR/栈帧可定位中断号） */
void Default_Handler(void)
{
while (1) {}
}
