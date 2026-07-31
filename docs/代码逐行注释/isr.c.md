# isr.c 逐行注释

> 行号与源文件一致。

TC264 中断向量实现：PIT 清标志、EXTI 清标志（VSYNC 调 `camera_vsync_handler`）、DMA 调 `camera_dma_handler`、UART RX/ER 分发。每个 ISR 入口先 `interrupt_global_enable(0)` 允许嵌套更高优先级中断。

**说明**：工程中无 `isr.h`，向量名与优先级由 `isr_config.h` + `IFX_INTERRUPT` 宏注册。

## 处理器分工

| ISR | 作用 |
|-----|------|
| `exti_ch3_ch7_isr` | P02_0 VSYNC → `camera_vsync_handler()` |
| `dma_ch5_isr` | 帧 DMA 完成 → `camera_dma_handler()` |
| `uart1_rx_isr` | 摄像头串口 → `camera_uart_handler()` |
| `uart0_rx_isr` | 可选调试串口中断 |
| CCU6 PIT | 仅清标志，业务在别处轮询/回调 |
| 其它 EXTI/UART | 占位清标志或错误处理 |

```c
// L1: 文件头注释
/* isr.c */
// L2: 空行

// L3: 中断优先级与服务目标宏
#include "isr_config.h"
// L4: 逐飞库：pit_clear_flag、exti_*、IfxAsclin 等
#include "zf_common_headfile.h"
// L5: 空行

// L6: CCU6_0 通道 0 PIT 中断；优先级 CCU6_0_CH0_ISR_PRIORITY(30)
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
// L7: 左花括号
{
// L8: 重新使能全局中断，允许高优先级抢占
    interrupt_global_enable(0);
// L9: 清除 CCU60 CH0 定时器中断标志
    pit_clear_flag(CCU60_CH0);
// L10: ISR 结束
}
// L11: 空行

// L12: CCU6_0 CH1 PIT；优先级 31
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
// L13: 左花括号
{
// L14: 允许嵌套
    interrupt_global_enable(0);
// L15: 清 CH1 标志
    pit_clear_flag(CCU60_CH1);
// L16: ISR 结束
}
// L17: 空行

// L18: CCU6_1 CH0；优先级 32
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
// L19: 左花括号
{
// L20: 允许嵌套
    interrupt_global_enable(0);
// L21: 清 CCU61 CH0
    pit_clear_flag(CCU61_CH0);
// L22: ISR 结束
}
// L23: 空行

// L24: CCU6_1 CH1；优先级 33
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
// L25: 左花括号
{
// L26: 允许嵌套
    interrupt_global_enable(0);
// L27: 清 CCU61 CH1
    pit_clear_flag(CCU61_CH1);
// L28: ISR 结束
}
// L29: 空行

// L30: EXTI 通道 0 与 4 共用向量；优先级 40
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
// L31: 左花括号
{
// L32: 允许嵌套
    interrupt_global_enable(0);
// L33: 若 P15_4 上 CH0 请求挂起
    if (exti_flag_get(ERU_CH0_REQ0_P15_4))
// L34: 左花括号
    {
// L35: 清除该 EXTI 标志
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
// L36: if 结束
    }
// L37: 若 P15_5 上 CH4 请求挂起
    if (exti_flag_get(ERU_CH4_REQ13_P15_5))
// L38: 左花括号
    {
// L39: 清除 CH4 标志
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
// L40: if 结束
    }
// L41: ISR 结束（无用户回调）
}
// L42: 空行

// L43: EXTI CH1 + CH5；优先级 41
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
// L44: 左花括号
{
// L45: 允许嵌套
    interrupt_global_enable(0);
// L46: P14_3 CH1
    if (exti_flag_get(ERU_CH1_REQ10_P14_3))
// L47: 左花括号
    {
// L48: 清标志
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
// L49: if 结束
    }
// L50: P15_8 CH5
    if (exti_flag_get(ERU_CH5_REQ1_P15_8))
// L51: 左花括号
    {
// L52: 清标志
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
// L53: if 结束
    }
// L54: ISR 结束
}
// L55: 空行

// L56: EXTI CH3 + CH7；含摄像头 VSYNC；优先级 43
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
// L57: 左花括号
{
// L58: 允许嵌套
    interrupt_global_enable(0);
// L59: P02_0：摄像头垂直同步线
    if (exti_flag_get(ERU_CH3_REQ6_P02_0))
// L60: 左花括号
    {
// L61: 先清硬件标志，避免重入
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
// L62: 通知摄像头驱动开始新帧采集
        camera_vsync_handler();
// L63: if 结束
    }
// L64: P15_1 CH7 请求
    if (exti_flag_get(ERU_CH7_REQ16_P15_1))
// L65: 左花括号
    {
// L66: 仅清标志
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
// L67: if 结束
    }
// L68: ISR 结束
}
// L69: 空行

// L70: DMA 通道 5 完成；优先级 60
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
// L71: 左花括号
{
// L72: 允许嵌套
    interrupt_global_enable(0);
// L73: 摄像头 DMA 行/帧缓冲搬运完成处理
    camera_dma_handler();
// L74: ISR 结束
}
// L75: 空行

// L76: UART0 发送完成；优先级 11
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
// L77: 左花括号
{
// L78: 允许嵌套
    interrupt_global_enable(0);
// L79: 空 ISR（库可能轮询或 DMA）
}
// L80: 空行

// L81: UART0 接收；优先级 10
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
// L82: 左花括号
{
// L83: 允许嵌套
    interrupt_global_enable(0);
// L84: 条件编译：调试串口中断接收
#if DEBUG_UART_USE_INTERRUPT
// L85: 调试协议解析（拼写为 interrupr）
    debug_interrupr_handler();
// L86: 结束条件编译
#endif
// L87: ISR 结束
}
// L88: 空行

// L89: UART1 TX；优先级 13
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
// L90: 左花括号
{
// L91: 允许嵌套
    interrupt_global_enable(0);
// L92: 空 ISR
}
// L93: 空行

// L94: UART1 RX；摄像头串口数据
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
// L95: 左花括号
{
// L96: 允许嵌套
    interrupt_global_enable(0);
// L97: 摄像头 UART 协议字节处理
    camera_uart_handler();
// L98: ISR 结束
}
// L99: 空行

// L100: UART2 TX；优先级 16
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
// L101: 左花括号
{
// L102: 允许嵌套
    interrupt_global_enable(0);
// L103: 空 ISR
}
// L104: 空行

// L105: UART2 RX；优先级 17
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
// L106: 左花括号
{
// L107: 允许嵌套
    interrupt_global_enable(0);
// L108: 空 ISR
}
// L109: 空行

// L110: UART3 TX；优先级 19
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
// L111: 左花括号
{
// L112: 允许嵌套
    interrupt_global_enable(0);
// L113: 空 ISR
}
// L114: 空行

// L115: UART3 RX；优先级 20
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
// L116: 左花括号
{
// L117: 允许嵌套
    interrupt_global_enable(0);
// L118: 空 ISR
}
// L119: 空行

// L120: UART0 错误中断；优先级 12
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
// L121: 左花括号
{
// L122: 允许嵌套
    interrupt_global_enable(0);
// L123: 逐飞 ASC 驱动错误恢复
    IfxAsclin_Asc_isrError(&uart0_handle);
// L124: ISR 结束
}
// L125: 空行

// L126: UART1 错误；优先级 15
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
// L127: 左花括号
{
// L128: 允许嵌套
    interrupt_global_enable(0);
// L129: UART1 错误处理
    IfxAsclin_Asc_isrError(&uart1_handle);
// L130: ISR 结束
}
// L131: 空行

// L132: UART2 错误；优先级 18
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
// L133: 左花括号
{
// L134: 允许嵌套
    interrupt_global_enable(0);
// L135: UART2 错误处理
    IfxAsclin_Asc_isrError(&uart2_handle);
// L136: ISR 结束
}
// L137: 空行

// L138: UART3 错误；优先级 21
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
// L139: 左花括号
{
// L140: 允许嵌套
    interrupt_global_enable(0);
// L141: UART3 错误处理
    IfxAsclin_Asc_isrError(&uart3_handle);
// L142: ISR 结束
}
```
