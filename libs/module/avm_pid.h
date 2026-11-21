#ifndef _AVM_PID_H_
#define _AVM_PID_H_
/*
 * ============================================================================
 * 【中文导读】avm_pid.h —— 串级 PID 的参数表与运行时结构体
 * ----------------------------------------------------------------------------
 * 本文件只放"数据"，算法在 avm_pid.c：
 *   - OUTTER_LOOP_Kx：角度外环三参数（输入=角度误差，输出=期望角速度修正量）
 *   - INNER_LOOP_Kx ：角速度内环三参数（输入=角速度误差，输出=PWM 增量）
 *   - PID_IMAX/IMIN ：外环积分限幅（anti-windup 抗积分饱和）
 *   - pid_st        ：单轴 PID 运行时状态（历史值 + 反馈指针 + 输出通道指针）
 *
 * 内环 KI=0 是刻意设计：角速度环要求"快"，纯 PD 响应快不超调；
 * 积分只留在外环慢慢消除重心偏差引起的稳态倾斜（详见 Docs/05-串级PID.md）。
 * 调参就改这几个宏，改完重新编译烧录（Avem 没有在线调参存储）。
 * ============================================================================
 */

/* ---- 外环（角度环）参数：决定"回平多快、会不会慢悠悠晃" ---- */
#define OUTTER_LOOP_KP 3 //0.257 * 0.83 0.255
#define OUTTER_LOOP_KI 1
#define OUTTER_LOOP_KD 0.3f

/* ---- 内环（角速度环）参数：决定"抗扰多硬、会不会抽搐" ---- */
#define INNER_LOOP_KP 1
#define INNER_LOOP_KI 0
#define INNER_LOOP_KD 0.3f

/* 误差总和限幅（SUM_ERRO_x）：预留的旧版限幅参数，当前算法未引用，
 * 实际生效的是下面的 PID_IMAX/IMIN */
#define SUM_ERRO_MAX 900
#define SUM_ERRO_MIN -900

/* 外环积分项限幅：积分每周期累加 Error，长时间静差会越积越大，
 * 钉在 ±30 以内防止"积分饱和"（起飞瞬间反向猛烈修正） */
#define PID_IMAX 30
#define PID_IMIN -30

/* 单轴 PID 运行时状态：Roll/Pitch 各需一份（本工程只实例化了 Roll=avm_pid）。
 * 指针成员在初始化时绑定数据源与输出端，是"传感器→控制→执行器"的接线处：
 *   InnerLast/OutterLast —— 上周期角速度/角度，供内环/外环 D 项做后向差分
 *   Feedback             —— 指向姿态解算输出 g_Roll（avm_mpu6050.c 更新）
 *   Gyro                 —— 指向本轴角速度 avm_euler.gX（°/s）
 *   Error/p/i/d          —— 本周期计算的中间量（p 复用为内环比例项，语义见 .c）
 *   output               —— PID 输出（PWM 增量，先当外环输出=期望角速度用）
 *   Channel1/Channel2    —— 指向 TIM3->CCR2/CCR4：本轴两颗电机的比较寄存器 */
typedef struct {
    float InnerLast;
    float OutterLast;
    float *Feedback;
    float *Gyro;
    float Error;
    float p;
    float i;
    float d;
    short output;
    __IO uint16_t *Channel1;
    __IO uint16_t *Channel2;
} pid_st, *pid_pst;

/* 全局唯一的 PID 实例（默认绑定 Roll 轴，字段绑定见 avm_pid.c 初始化列表） */
extern pid_st avm_pid;

/* 单轴串级 PID 一步：setPoint=目标角度（度），每周期由控制流调用一次 */
void pid_SingleAxis(pid_pst package, float setPoint);
/* 预留的双轴接口（当前版本未实现，调用点空缺） */
void pid(float setPoint, float d);

#endif
