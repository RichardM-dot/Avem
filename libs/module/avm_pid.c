/*
 * ============================================================================
 * 【中文导读】avm_pid.c —— 单轴串级 PID 控制器（本飞控的灵魂）
 * ----------------------------------------------------------------------------
 * 本文件做什么：
 *   实现一个"角度外环 PID + 角速度内环 PD"的串级控制器（cascade PID），
 *   每根轴（Roll/Pitch）一套，由主循环按固定周期调用。输出直接叠加到
 *   两个电机的 PWM 占空比上（差动控制：一个加、一个减）。
 *
 * 串级 PID 为什么这么设计（面试/实操高频知识点）：
 *   - 外环输入 = 目标角度 与 实际角度(融合后的 Roll) 之差，输出"期望角速度"；
 *     积分项 I 只放外环，用于消除重心偏差导致的稳态倾斜。
 *   - 内环输入 = 期望角速度(外环输出) 与 陀螺仪实测角速度 之差，输出 PWM 增量；
 *     角速度变化快，用纯 PD 就够，响应快、不超调。
 *   - 简单说：外环管"稳"，内环管"快"。
 *
 * 关键函数：
 *   pid_SingleAxis(): 单轴完整控制一步（外环 PID → 内环 PD → 输出限幅混控）
 *
 * 调参入口：OUTTER_LOOP_Kx / INNER_LOOP_Kx / PID_IMAX（见 avm_pid.h）
 * ============================================================================
 */

#include <avm_core.h>

// float InnerLast;			//保存内环旧值以便后向差分
// float OutterLast;		//保存外环旧值以便后向差分
// float *Feedback;			//反馈数据, 实时的角度数据
// float *Gyro;				//角速度
// float Error;				//误差值
// float p;					//比例项(内环外环共用)
// float i;					//积分项(仅用于外环)
// float d;					//微分项(内环外环共用)
// short output;			//PID输出, 用来修改PWM值, 2字节
// __IO uint16_t *Channel1;	//PWM输出, 通道1
// __IO uint16_t *Channel2;	//PWM输出, 通道2

/* 单轴 PID 实例：默认绑定 Roll 轴。
 * Feedback 指向姿态融合算出的 g_Roll（avm_mpu6050.c 里更新）；
 * Channel1/Channel2 指向 TIM3 的两个比较寄存器（MOTOR2/MOTOR4，电机对）；
 * Gyro 指向陀螺仪 X 轴角速度——Roll 轴的转动由 X 轴角速度描述。 */
pid_st avm_pid = {
    .InnerLast  = 0,
    .OutterLast = 0,
    .Feedback   = &g_Roll,
    .Channel1   = &MOTOR2,
    .Channel2   = &MOTOR4,
    .Gyro       = &avm_euler.gX,
};

/*
 * 单轴串级 PID 一步。
 * 参数：temp    = pid 实例（本机只用了全局 avm_pid）
 *       setPoint = 目标角度（遥控器输入映射而来，0 表示悬停水平）
 */
void pid_SingleAxis(pid_pst temp, float setPoint) {
    /* ---- 外环（角度环）---- */
    temp->Error = *temp->Feedback - setPoint;      // 误差 = 实际角度 - 目标角度
//Outter Loop PID
    /* 积分累积误差，并做积分限幅（anti-windup 抗饱和）：
     * 若不限幅，起飞前长时间有静差会让积分爆表，起飞瞬间猛烈修正（俗称积分饱和） */
    temp->i += temp->Error;
    if (temp->i > PID_IMAX)
		temp->i = PID_IMAX;
    else if (temp->i < PID_IMIN)
		temp->i = PID_IMIN;

    /* 微分项用"本次角度 - 上次角度"的后向差分近似（不用真实导数，抗噪声） */
    temp->d = *temp->Feedback - temp->OutterLast;

    /* 外环输出 = 期望的角速度修正量（Kp/Ki/Kd 是外环三参数） */
    temp->output = (short)(OUTTER_LOOP_KP * (temp->Error) + OUTTER_LOOP_KI * temp->i + OUTTER_LOOP_KD * temp->d);
    temp->OutterLast = *temp->Feedback; //Save Old Data
//Inner Loop PD
    /* ---- 内环（角速度环，纯 PD）---- */
    /* p 这里语义是"期望角速度 + 实测角速度 = 角速度误差"（作者复用变量名，注意别混淆）：
     * 期望角速度即外环输出 output，实测角速度取负号即 +Gyro 的效果 */
    temp->p = temp->output + *temp->Gyro;
    /* 内环 D：角速度的后向差分（本次 - 上次） */
    temp->d = *temp->Gyro - temp->InnerLast;
    /* 内环输出 = 最终 PWM 增量 */
    temp->output = (short)(INNER_LOOP_KP * temp->p + INNER_LOOP_KD * temp->d);

    /* ---- 差动混控与限幅 ----
     * 通道1 加 output、通道2 减 output：同轴上两颗电机反向增减即产生滚转力矩。
     * 两次限幅把占空比钉死在 [THROTTLE_MIN, THROTTLE_MAX]，
     * 防止输出越界导致电机满转/停转失控。 */
    if (*temp->Channel1+temp->output > THROTTLE_MAX)
		*temp->Channel1 = THROTTLE_MAX;
    else if (*temp->Channel1+temp->output < THROTTLE_MIN)
		*temp->Channel1 = THROTTLE_MIN;
    else
		*temp->Channel1 += (short)temp->output;

    if (*temp->Channel2-temp->output > THROTTLE_MAX)
		*temp->Channel2 = THROTTLE_MAX;
    else if
		(*temp->Channel2-temp->output < THROTTLE_MIN) *temp->Channel2 = THROTTLE_MIN;
    else
		*temp->Channel2 -= (short)temp->output;

    /* 保存本次角速度，供下一周期内环 D 项做差分 */
    temp->InnerLast = *temp->Gyro;
}
