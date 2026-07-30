/* telemetry.c - 逐飞无线串口 CSV 遥测(有界队列,不阻塞主循环) */
#include "seekfree_baud.h"
#include <stdio.h>
#include "config.h"
#include "control.h"
#include "image.h"
#include "telemetry.h"
#include "zf_common_headfile.h"

volatile uint8_t telemetry_enable = 0;

#define TELEM_QUEUE_BYTES   (384u)
#define TELEM_PUMP_BUDGET   (64u)
#define TELEM_HW_CHUNK      (30u)

static uint8_t  s_queue[TELEM_QUEUE_BYTES];
static uint16_t s_head;
static uint16_t s_tail;
static uint16_t s_count;
static uint32_t s_frame;
static uint8_t  s_need_header;
static uint8_t  s_prev_enable;

static uint16_t queue_free(void)
{
    return (uint16_t)(TELEM_QUEUE_BYTES - s_count);
}

static uint8_t queue_push(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0 || len == 0u)
    {
        return 0u;
    }
    if (len > queue_free())
    {
        return 0u;
    }
    for (i = 0; i < len; i++)
    {
        s_queue[s_head] = data[i];
        s_head = (uint16_t)((s_head + 1u) % TELEM_QUEUE_BYTES);
    }
    s_count = (uint16_t)(s_count + len);
    return 1u;
}

static void queue_push_line(const char *line)
{
    uint16_t len = 0u;
    while (line[len] != '\0')
    {
        len++;
    }
    (void)queue_push((const uint8_t *)line, len);
}

void telemetry_init(void)
{
    (void)wireless_uart_init();
    s_head = 0u;
    s_tail = 0u;
    s_count = 0u;
    s_frame = 0u;
    s_need_header = 1u;
    s_prev_enable = 0u;
}

void telemetry_pump(void)
{
    uint16_t budget = TELEM_PUMP_BUDGET;
    uint8_t chunk[TELEM_HW_CHUNK];
    uint16_t i;

    while (s_count > 0u && budget > 0u)
    {
        uint16_t n;

        if (gpio_get_level(WIRELESS_UART_RTS_PIN))
        {
            break;
        }

        n = s_count;
        if (n > budget)
        {
            n = budget;
        }
        if (n > TELEM_HW_CHUNK)
        {
            n = TELEM_HW_CHUNK;
        }

        for (i = 0; i < n; i++)
        {
            chunk[i] = s_queue[s_tail];
            s_tail = (uint16_t)((s_tail + 1u) % TELEM_QUEUE_BYTES);
        }
        s_count = (uint16_t)(s_count - n);
        uart_write_buffer(WIRELESS_UART_INDEX, chunk, n);
        budget = (uint16_t)(budget - n);
    }
}

void telemetry_update(uint32_t t_ms, uint32_t frame,
                      const track_info_t *ti, const control_out_t *out)
{
    char buf[128];
    int len;

    s_frame = frame;
    if (!telemetry_enable)
    {
        s_prev_enable = 0u;
        return;
    }

    if (!s_prev_enable)
    {
        s_need_header = 1u;
    }
    s_prev_enable = 1u;

    if (s_need_header)
    {
        /* hold: 0=err 为实测,>0=盲区保持中(值为已保持帧数),见 image.h err_hold
           tgt : slew 之前的目标占空比 = 菜单 Duty。dty<tgt 只可能是还在爬升,
                 因为行数限速(ROWS_CAP)已删,不存在其他把 duty 压下去的机制。
                 保留这一列的意义是让日志自带"当时命令的是多少速度" */
        queue_push_line("# t_ms,err,hold,srv,dty,tgt,row,lst,th,crs,kp,kd,wref\r\n");
        s_need_header = 0u;
    }

    if ((frame % TELEM_DIV) != 0u)
    {
        return;
    }

    len = snprintf(buf, sizeof(buf),
                   "%lu,%d,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%u\r\n",
                   (unsigned long)t_ms,
                   (int)out->error_used,
                   (unsigned)ti->err_hold,
                   (unsigned)out->servo_pwm,
                   (unsigned)out->duty,
                   (unsigned)out->duty_target,
                   (unsigned)ti->valid_rows,
                   (unsigned)ti->both_lost_rows,
                   (unsigned)ti->threshold,
                   (unsigned)ti->cross_valid,
                   (double)steer_kp,
                   (double)steer_kd,
                   (unsigned)steer_w_duty_ref);
    if (len > 0)
    {
        uint16_t send_len = (len < (int)sizeof(buf))
                          ? (uint16_t)len
                          : (uint16_t)(sizeof(buf) - 1u);
        (void)queue_push((const uint8_t *)buf, send_len);
    }
}
