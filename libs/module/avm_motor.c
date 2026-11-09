/*
 * ============================================================================
 * 【中文导读】avm_motor.c —— 四路电机 PWM 驱动（TIM3）
 * ----------------------------------------------------------------------------
 * 本文件做什么：
 *   用 STM32F103 的 TIM3 高级/通用定时器产生 4 路标准 50Hz~500Hz PWM，
 *   直接驱动空心杯电调/电调板。寄存器级配置（不使用 HAL），是学习
 *   "STM32 定时器 PWM 从零配置"的最小完整示例。
 *
 * 在整体架构中的位置：
 *   模块池 [3]，由 avm_core 统一初始化（MOTOR_SETTING 宏里调 motor_PWM_Init）。
 *   PID 输出写入的 MOTOR1~MOTOR4 宏即 TIM3->CCR1~CCR4 比较寄存器（见 avm_motor.h）。
 *
 * PWM 三要素（读下面寄存器配置时对照）：
 *   - PSC 预分频：72MHz/(PSC) 得计数时钟
 *   - ARR 自动重装值：计数周期，决定 PWM 分辨率
 *   - CCR 比较值：CNT 数到 CCR 翻转输出，CCR/ARR 即占空比
 * ============================================================================
 */

#include <avm_core.h>

static unsigned char avm_motor_init(void *);

/* 电机模块的注册信息（模块池 [3]，init 时执行 MOTOR_SETTING 完成全部配置） */
avm_module_t avm_motor_module_st = {
    0,
    NULL,
    avm_motor_init,
    NULL,
    NULL
};

static unsigned char avm_motor_init(void *arg) {
    MOTOR_SETTING();     /* 宏展开：调 motor_PWM_Init 设定 PWM 频率（见 avm_motor.h） */
    return 0;
}

//A7
/*
 * TIM3 四通道 PWM 初始化（PA6/PA7 + PB0/PB1 对应 CH1~CH4）。
 * 参数：arr = 自动重装值（决定 PWM 周期分辨率）
 *       psc = 预分频值（72MHz/psc = 计数频率）
 * 例：arr=100, psc=144 → 500Hz PWM、100 级油门分辨率
 */
void motor_PWM_Init(unsigned short arr, unsigned short psc) {
    //RCC->APB1ENR |= 1<<1;       //TIM3 enable
    //RCC->APB2ENR |= 1<<2;       //GPIOA enable

    /* 1. 打开时钟：TIM3 在 APB1 总线，GPIOA/B 在 APB2 总线（不开时钟寄存器写不进去） */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;		//TIM3 Enable
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;		//IO Port A Enable
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;		//IO Port B for TIM3 Channel 3 and 4

    /* 2. GPIO 配置：CRL 每引脚 4 位，0xB = 复用推挽输出 50MHz（定时器接管引脚）。
     *    先清后置：PA6/PA7 的高 8 位清零再写 0xBB；PB0/PB1 同理写低 8 位 */
    GPIOA->CRL &= 0x00FFFFFF;   //Clean
    GPIOA->CRL |= 0xBB000000;   //复用推挽输出
    GPIOA->ODR |= 1<<7;			//CH2 GPIO Config
    GPIOA->ODR |= 1<<6;			//CH1 GPIO Config

    GPIOB->CRL &= 0xFFFFFF00;
    GPIOB->CRL |= 0x000000BB;
    GPIOB->ODR |= 1;
    GPIOB->ODR |= 1<<1;

    /* 3. 时基：ARR/PSC 写入的都是"实际值-1"（寄存器从 0 开始计数） */
    TIM3->ARR = arr - 1;
    TIM3->PSC = psc - 1;

    /* 4. 捕获比较模式：OCxM=110 即 PWM 模式1（CNT<CCR 输出有效电平）；
     *    OCxPE=1 使能预装载（CCR 的修改在下个周期才生效，避免波形毛刺） */
    TIM3->CCMR1 |= 6<<4;    //CH1 Set OC1M[2:0]: PWM Mode
    TIM3->CCMR1 |= 1<<3;    //CH1 Set OC1PE: Enable
    TIM3->CCMR1 |= 6<<12;   //CH2 Set OC2M[2:0]: PWM Mode
    TIM3->CCMR1 |= 1<<11;   //CH2 Set OC2PE: Enable
    TIM3->CCMR2 |= 6<<4;    //CH3 Set OC3M[2:0]: PWM Mode
    TIM3->CCMR2 |= 1<<3;    //CH3
    TIM3->CCMR2 |= 6<<12;   //CH4
    TIM3->CCMR2 |= 1<<11;

    /* 5. CCER：使能 4 个通道的输出到引脚（bit0/4/8/12 = CC1E~CC4E） */
    TIM3->CCER |= 1;        //CH1 Output Enable
    TIM3->CCER |= 1<<4;     //CH2 Output Enable
    TIM3->CCER |= 1<<8;
    TIM3->CCER |= 1<<12;

    /* 6. 启动：ARPE=1 使 ARR 预装载（改频率也原子生效）；CEN=1 开始计数 */
    TIM3->CR1 = 0x80;       //APRE Enable
    TIM3->CR1 |= 1;         //Set CEN, Allow to Count
    //TIM1->BDTR |= 1<<15;  //高级定时器需要使能BDTR寄存器
}
