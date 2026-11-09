/*
 * ============================================================================
 * 【中文导读】main.c —— Avem 飞控程序入口
 * ----------------------------------------------------------------------------
 * 裸机程序的最小主函数：
 *   1. 调 avm_core_module_st.init_module() —— 触发模块池按顺序初始化
 *      UART → I2C → 电机 PWM → MPU6050（链路见 avm_core.c）。
 *   2. 进入空转死循环 —— 真正的控制逻辑在别处周期执行（本版本的控制/解算
 *      调用点由工程中断或后续版本接入；姿态解算函数 IMU_Comput 与
 *      pid_SingleAxis 已就绪，见 avm_mpu6050.c / avm_pid.c）。
 *
 * 文件里被注释掉的代码是作者的调试痕迹：MPU 自检、串口输出、电机自转测试，
 * 保留它们正好可以对照学习"裸机项目的调试手法"。
 * ============================================================================
 */

#include <avm_core.h>

int main() {
	// #ifdef DEBUG_BLDC
	// 	//Brushless motor auto init
	//     MOTOR_SETTING();
	// #endif
	//
    // uart_sendStr("Config MPU6050...");
    // UART_CR();
    // MPU_init();
    // uart_sendStr("MPU6050 Connect Success!");
    // UART_CR();
	/* 一句话启动整个系统：core 模块按模块池顺序初始化其余全部模块 */
	avm_core_module_st.init_module(NULL);
	// avm_uart_module_st.init_module(avm_uart_module_st.confs);
	// uart_init(72, 115200);

	while(1) {
	}
}
