/* seekfree_baud.h - 无线串口波特率覆盖
 *
 * 须在编译 zf_device_wireless_uart.c 之前生效,推荐在 ADS 工程添加预处理器宏:
 *   WIRELESS_UART_BUAD_RATE=460800
 * 或在 libraries/zf_device/zf_device_wireless_uart.h 中把
 *   WIRELESS_UART_BUAD_RATE 改为 (460800) 后全量重编译。
 *
 * PC 端逐飞无线串口助手也需设为相同波特率。 */
#ifndef SEEKFREE_BAUD_H
#define SEEKFREE_BAUD_H

#include "config.h"

#ifndef WIRELESS_UART_BUAD_RATE
#define WIRELESS_UART_BUAD_RATE (TELEM_UART_BAUD)
#endif

#endif /* SEEKFREE_BAUD_H */
