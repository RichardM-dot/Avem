#ifndef _AVM_MOTOR_H_
#define _AVM_MOTOR_H_
/*
 * ============================================================================
 * 【中文导读】avm_motor.h —— 电机通道宏、油门界限与启动宏（TIM3 PWM）
 * ----------------------------------------------------------------------------
 * 职责：
 *   1. MOTOR1~MOTOR4 宏把 TIM3 四个比较寄存器 CCR1~CCR4 映射成"电机油门变量"，
 *      写 MOTOR2 = x 即设定 2 号电机占空比——PID 混控代码因此一目了然。
 *   2. THROTTLE_MAX/MIN/MID：油门上下限（单位=定时器计数），PID 输出限幅
 *      和电调行程标定都用它。
 *   3. MOTOR_SETTING()：模块池初始化时执行的"上电序列"（先设 PWM 再按
 *      电调规程给油门行程）。
 *
 * 数据流位置：pid_SingleAxis() 写 *Channel1/*Channel2（即 MOTOR2/MOTOR4）
 *             → TIM3 硬件自动输出 PWM → 电调 → 电机转速。
 * ============================================================================
 */

extern avm_module_t avm_motor_module_st;

/* 电调/电调板的"普通启动"模式开关（见下方 MOTOR_SETTING 两个分支） */
#define MOTOR_NORMAL_STARTUP


/* 四路电机 = TIM3 四个捕获/比较寄存器。引脚对应：CH1=PA6、CH2=PA7、
 * CH3=PB0、CH4=PB1。写入值的含义：占空比 = CCR / (ARR+1)，见 motor_PWM_Init */
#define MOTOR1 TIM3->CCR1
#define MOTOR2 TIM3->CCR2
#define MOTOR3 TIM3->CCR3
#define MOTOR4 TIM3->CCR4

/* 四路一起赋值（利用宏展开成连续赋值语句），用于上电油门序列 */
#define MOTOR_ALL MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4

/* 油门界限（单位=TIM3 计数值。MOTOR_SETTING 配置 ARR=36000/PSC=40 时，
 * 计数时钟 = 72MHz/40 = 1.8MHz）：
 * MAX=3600 计数 = 2ms 高电平；MIN=1620 = 0.9ms——即普通 50Hz 舵机/电调
 * 协议的行程范围。PID 输出被限幅在 [MIN, MAX] 内，电机永远不会满转/停转 */
#define THROTTLE_MAX (unsigned short)3600	//2ms	-	top position of throttle
#define THROTTLE_MIN (unsigned short)1620	//0.9ms	-	bottom position of throttle
#define THROTTLE_MID (unsigned short)1650

/*
 * 上电油门序列宏（模块池初始化时执行一次）。
 * 注意：宏体是多行 \ 续行定义，按规范禁止在宏体内部插入注释！
 * 两个分支都模拟"电调油门行程标定"：
 *   普通启动（当前启用）：PWM 500Hz 就绪 → 四路置 MIN（怠速位）→ 等 4s
 *     让电调完成自检（听"哔-哔"确认音）→ 置 MID（略高于怠速）待命。
 *   备用分支（MAX 起步）：3s 最高油门 + 8s 最低油门——兼容某些要求
 *     "满油门上电做行程校准"的电调，标定完回落怠速。
 */
#ifdef MOTOR_NORMAL_STARTUP
    #define MOTOR_SETTING() do {\
        motor_PWM_Init(36000,40);\
		MOTOR_ALL = THROTTLE_MIN;\
        delay_ms(4000);\
		MOTOR_ALL = THROTTLE_MID;\
    } while(0)
#else
    #define MOTOR_SETTING() do {\
        motor_PWM_Init(36000,40);\
		MOTOR_ALL = THROTTLE_MAX;\
        delay_ms(3000);\
		MOTOR_ALL = THROTTLE_MIN;\
        delay_ms(8000);\
    } while(0)
#endif


/* TIM3 四通道 PWM 初始化：arr=自动重装值、psc=预分频值（均写实际值-1）。
 * PWM 频率 = 72MHz / ((arr+1)*(psc+1))；本工程 36000/40 → 50Hz（36000 级分辨率，
 * 50Hz 正是普通电调协议频率，与 2ms/0.9ms 的油门界限自洽） */
void motor_PWM_Init(unsigned short arr, unsigned short psc);  //72MHz / (arr + 1)*(psc + 1)

#endif
