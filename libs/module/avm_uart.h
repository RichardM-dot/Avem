#ifndef _AVM_UART_H_
#define _AVM_UART_H_
/*
 * 【中文导读】avm_uart.h —— USART1 调试/控制链路接口（B级：文件头导读）
 * --------------------------------------------------------------
 * 职责：串口驱动对外接口 + 上位机协议约定。
 * 主要函数（实现见 avm_uart.c，寄存器级 USART1@PA9/PA10）：
 *   uart_init(pclk2, bound) —— 波特率/GPIO/中断配置
 *   uart_sendData/sendStr   —— 阻塞发送单字节/字符串
 *   uart_Float2Char 等      —— 浮点/整型转十进制字符串发送（无 printf 的裸机方案）
 *   USART1_IRQHandler       —— 接收中断：上位机 '$' 命令触发一次完整控制循环
 * 协议约定：
 *   TOKEN '$'  —— 请求一帧数据：读 MPU→解算→PID→回传 Pitch/Roll/Yaw/电机值
 *   '>'        —— 请求跳转内置 Bootloader（ISP 烧录模式）
 *   数据帧     —— "@" 分隔、"\n\r" 结尾的 ASCII 文本行，MultiWii 类上位机可解析
 * 在数据流中的位置：整个飞控的"发动机点火开关"——控制循环由串口命令驱动。
 */


void uart_init(unsigned int pclk2, unsigned int bound);
void uart_sendData(unsigned char data);
void uart_short2char(short k);
void uart_int2char(unsigned int k);
unsigned char uart_Float2Char(float value);
void uart_sendStr(char *cmd);
void USART1_IRQHandler(void);

extern avm_module_t avm_uart_module_st;

#define UART_CR() do {\
    uart_sendData(0x0D);\
    uart_sendData(0x0A);\
} while(0)

#define UART_CLEAR() uart_sendStr("\033[H\033[J")

#define TOKEN           '$'

#endif
