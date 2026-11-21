#ifndef _AVM_WIFI_H_
#define _AVM_WIFI_H_
/*
 * 【中文导读】avm_wifi.h —— ESP8266 AT 指令链路接口（B级：文件头导读）
 * --------------------------------------------------------------
 * 职责：预留的 Wi-Fi 数传扩展接口，走 USART3@PB10/PB11 与 ESP8266 通信
 * （AT+CWMODE=2 建 AP、CIPSERVER 开 TCP 服务）。实现见 avm_wifi.c。
 * 注意 WIFI_PCLK2=36：USART3 挂 APB1（36MHz），与 USART1 的 PCLK2=72MHz
 * 不同——两份驱动算波特率时的基准时钟不一样，抄代码时最易踩这个坑。
 * 本模块未挂入 avm_modules_pool，当前工程默认不启用。
 */
//Only USART1 PCLK2 = 72
#define WIFI_PCLK2 36
#define WIFI_BOUND 115200

void wifi_init();
void wifi_sendSingleByte(unsigned char cmd);
void wifi_sendData(char * cmd);
void wifi_sendCmd(char * cmd);
void wifi_Config();
extern void delay(volatile unsigned int count);
#endif
