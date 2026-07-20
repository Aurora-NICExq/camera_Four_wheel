/*********************************************************************************************************************
 * 模块：display.c — 无线串口遥测 + 蜂鸣器提示音
 *
 * 屏幕归属：IPS200 现由调试菜单常驻独占（menu.c / menu_port.c）——本模块不画屏，只做两件事：
 *   - 无线串口一行遥测（ENABLE_UART_TELEMETRY，内部按帧分频、非阻塞）；
 *   - 图像失效 / 相机看门狗触发时的蜂鸣器长鸣（帧计数控制时长，永不阻塞）。
 *
 * 精简版（直道+转弯）：无状态机 → 遥测不再输出状态名/曲率；蜂鸣仅剩一种“故障长鸣”。
 * 硬规则：任何串口写入都不允许阻塞帧同步主循环。
 *
 * 使用的逐飞 API（已核对真实头文件签名）：
 *   ips200_init                                          （zf_device_ips200.h，仅上电做一次硬件初始化）
 *   wireless_uart_init / wireless_uart_send_buffer       （zf_device_wireless_uart.h）
 ********************************************************************************************************************/
#include <stdio.h>
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "image.h"
#include "control.h"
#include "motor.h"
#include "display.h"

/*===================================================================================================================
 * 内部状态
 *==================================================================================================================*/

static uint8_t  g_chirp_frames;     /* 蜂鸣器剩余鸣叫帧数 */

/*===================================================================================================================
 * 对外接口
 *==================================================================================================================*/

void display_init(void)
{
    /* 竖屏必须在 init 前设置方向，否则硬件 MADCTL 仍按库默认写死。 */
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_CONNECT_TYPE);
    ips200_clear();
    wireless_uart_init();
    g_chirp_frames = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * display_chirp_fault — 请求一次故障长鸣（图像失效 / 相机看门狗时由 cpu0_main 触发）。
 *------------------------------------------------------------------------------------------------------------------*/
void display_chirp_fault(void)
{
    g_chirp_frames = CHIRP_FRAMES_LONG;
}

/*-------------------------------------------------------------------------------------------------------------------
 * display_telemetry — 遥测一行文本。常规模式按 TELEMETRY_DIV 限流；
 * TEST_COAST 模式每帧输出 (frame, valid_rows, duty) —— 滑行标定的原始数据。
 *------------------------------------------------------------------------------------------------------------------*/
void display_telemetry(const track_info_t *ti, const control_out_t *out, uint32_t frame)
{
#if ENABLE_UART_TELEMETRY
    char buf[96];
    int len;
#if TEST_COAST
    /* 标定格式固定三列，便于上位机/脚本直接解析 */
    len = snprintf(buf, sizeof(buf), "%lu,%u,%u\r\n",
                   (unsigned long)frame, (unsigned)ti->valid_rows, (unsigned)out->duty);
    if (len > 0)
    {
        wireless_uart_send_buffer((const uint8 *)buf, (uint32)len);
    }
#else
    if ((frame % TELEMETRY_DIV) == 0u)
    {
        len = snprintf(buf, sizeof(buf), "F%lu E%d R%u D%u\r\n",
                       (unsigned long)frame, (int)out->error_used,
                       (unsigned)ti->valid_rows, (unsigned)out->duty);
        if (len > 0)
        {
            wireless_uart_send_buffer((const uint8 *)buf, (uint32)len);
        }
    }
#endif
#else
    (void)ti; (void)out; (void)frame;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * display_update — 每帧调用；只负责蜂鸣器提示音（不碰屏幕），绝不阻塞。
 *------------------------------------------------------------------------------------------------------------------*/
void display_update(void)
{
    if (g_chirp_frames > 0)
    {
        g_chirp_frames--;
        hal_buzzer_on();
    }
    else
    {
        hal_buzzer_off();
    }
}
