# isr_config.h 逐行注释

> 行号与源文件一致。

中断服务配置：为 CCU6 定时器、EXTI 外部中断、DMA、四路 UART 指定 **服务目标 CPU/DMA** 与 **NVIC 优先级**。`isr.c` 中 `IFX_INTERRUPT` 宏引用这些常量注册向量。

优先级数字越小越高（英飞凌惯例）。摄像头 VSYNC（EXTI CH3）=43，DMA=60，UART 在 10~21。

```c
// L1: 文件头注释
/* isr_config.h */
// L2: 头文件保护开始
#ifndef _isr_config_h
// L3: 定义保护宏
#define _isr_config_h
// L4: 空行

// L5: CCU6_0 通道 0 中断投递到 CPU0
#define CCU6_0_CH0_INT_SERVICE  IfxSrc_Tos_cpu0
// L6: PIT/定时器 CH0 优先级 30
#define CCU6_0_CH0_ISR_PRIORITY 30
// L7: CCU6_0 CH1 服务 CPU0
#define CCU6_0_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
// L8: CH1 优先级 31
#define CCU6_0_CH1_ISR_PRIORITY 31
// L9: CCU6_1 CH0 服务 CPU0
#define CCU6_1_CH0_INT_SERVICE  IfxSrc_Tos_cpu0
// L10: 优先级 32
#define CCU6_1_CH0_ISR_PRIORITY 32
// L11: CCU6_1 CH1 服务 CPU0
#define CCU6_1_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
// L12: 优先级 33
#define CCU6_1_CH1_ISR_PRIORITY 33
// L13: 空行

// L14: EXTI 通道 0 与 4 共用 ISR，服务 CPU0
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0
// L15: 优先级 40
#define EXTI_CH0_CH4_INT_PRIO    40
// L16: EXTI 通道 1 与 5 共用 ISR
#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0
// L17: 优先级 41
#define EXTI_CH1_CH5_INT_PRIO    41
// L18: EXTI 通道 2 与 6 投递 DMA（本工程 isr.c 未注册对应 ISR）
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma
// L19: DMA 侧优先级 5
#define EXTI_CH2_CH6_INT_PRIO    5
// L20: EXTI 通道 3 与 7：含摄像头 VSYNC P02_0
#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu0
// L21: 优先级 43，高于一般 UART
#define EXTI_CH3_CH7_INT_PRIO    43
// L22: 空行

// L23: DMA 完成中断服务 CPU0
#define DMA_INT_SERVICE         IfxSrc_Tos_cpu0
// L24: DMA 优先级 60（低于 VSYNC，避免嵌套过深）
#define DMA_INT_PRIO            60
// L25: 空行

// L26: UART0 中断服务 CPU0
#define UART0_INT_SERVICE       IfxSrc_Tos_cpu0
// L27: TX 优先级 11
#define UART0_TX_INT_PRIO       11
// L28: RX 优先级 10（略高于 TX）
#define UART0_RX_INT_PRIO       10
// L29: 错误中断 12
#define UART0_ER_INT_PRIO       12
// L30: 空行

// L31: UART1 服务 CPU0（摄像头串口 RX 在 isr.c 调 camera_uart_handler）
#define UART1_INT_SERVICE       IfxSrc_Tos_cpu0
// L32: TX 13
#define UART1_TX_INT_PRIO       13
// L33: RX 14
#define UART1_RX_INT_PRIO       14
// L34: ER 15
#define UART1_ER_INT_PRIO       15
// L35: 空行

// L36: UART2
#define UART2_INT_SERVICE       IfxSrc_Tos_cpu0
// L37: TX 16
#define UART2_TX_INT_PRIO       16
// L38: RX 17
#define UART2_RX_INT_PRIO       17
// L39: ER 18
#define UART2_ER_INT_PRIO       18
// L40: 空行

// L41: UART3
#define UART3_INT_SERVICE       IfxSrc_Tos_cpu0
// L42: TX 19
#define UART3_TX_INT_PRIO       19
// L43: RX 20
#define UART3_RX_INT_PRIO       20
// L44: ER 21
#define UART3_ER_INT_PRIO       21
// L45: 空行

// L46: 结束头文件保护
#endif
```
