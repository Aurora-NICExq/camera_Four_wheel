/* display.c - UART telemetry + buzzer chirps */
#include <stdio.h>
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "image.h"
#include "fsm.h"
#include "control.h"
#include "motor.h"
#include "display.h"

static uint8_t  g_chirp_frames;     /* 蜂鸣器剩余鸣叫帧数 */

void display_init(void)
{
    /* 竖屏必须在 init 前设置方向，否则硬件 MADCTL 仍按库默认写死。 */
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_CONNECT_TYPE);
    ips200_clear();
    wireless_uart_init();
    g_chirp_frames = 0;
}

void display_chirp_fault(void)
{
    g_chirp_frames = CHIRP_FRAMES_LONG;
}

void display_chirp(fsm_state_t state)
{
    switch (state)
    {
    case ST_CROSS:      g_chirp_frames = CHIRP_FRAMES_SHORT; break;
    case ST_RING_PRE:
    case ST_RING_IN:
    case ST_RING_EXIT:
    case ST_RAMP:
    case ST_RECOVERY:   g_chirp_frames = CHIRP_FRAMES_MID;   break;
    case ST_FAULT:      g_chirp_frames = CHIRP_FRAMES_LONG;  break;
    default: break;
    }
}

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
