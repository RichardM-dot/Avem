# Avem 注释范围

- A级（逐函数中文深注）：
  - src/main.c、src/startup.c（仅关键段：向量表结构、__Init_Data、Reset_Handler）
  - libs/module/avm_core.c/.h（模块池机制、delay、jump2ISP）
  - libs/module/avm_pid.c/.h（串级 PID 全部逻辑）
  - libs/module/avm_motor.c/.h（TIM3 PWM 配置）
  - libs/module/avm_mpu6050.c/.h（I2C 读写、姿态融合 Mahony 补偿滤波）
  - libs/module/avm_i2c.c（软件 I2C 时序）
- B级：avm_uart.c/.h、avm_wifi.c/.h（文件头导读块）
- C级：libs/STM32_USB-FS-Device_Lib_V4.0.0（ST 官方 USB 库，不动）
