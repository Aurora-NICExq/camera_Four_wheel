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

static uint8_t queue_push_line(const char *line)
{
    uint16_t len = 0u;
    while (line[len] != '\0')
    {
        len++;
    }
    return queue_push((const uint8_t *)line, len);
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

/* ---------------- 无线串口链路自检 ----------------
 * 走的是与 CSV 遥测完全相同的通路:同一条有界队列 + telemetry_pump 的
 * RTS 流控。不另开第二条 UART 写入路径——否则自检"通过"也不能证明
 * 遥测能通,那就失去意义了。
 * 返回 1 = 已入队;0 = 队列满被丢弃,说明链路没在排空
 * (RTS 被模块拉高,或 PC 侧没有接收) */
uint8_t telemetry_test_send(uint32_t seq, uint32_t t_ms)
{
    char buf[80];
    int  len = snprintf(buf, sizeof(buf),
                        "SEEKFREE WIRELESS TEST seq=%lu t=%lums baud=%lu\r\n",
                        (unsigned long)seq, (unsigned long)t_ms,
                        (unsigned long)TELEM_UART_BAUD);

    if (len <= 0)
    {
        return 0u;
    }
    if (len >= (int)sizeof(buf))
    {
        len = (int)sizeof(buf) - 1;
    }
    return queue_push((const uint8_t *)buf, (uint16_t)len);
}

/* 进入测试页时发一次。RULER 行是固定的可打印 ASCII 序列:
   波特率不匹配时它会变成乱码,比盯计数器更容易判断问题在哪一层 */
uint8_t telemetry_test_banner(void)
{
    uint8_t ok;

    ok  = queue_push_line("\r\n==== SEEKFREE WIRELESS UART TEST ====\r\n");
    ok &= queue_push_line("RULER 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n");
    return ok;
}

uint16_t telemetry_queue_depth(void)
{
    return s_count;
}

/* 1 = 模块正在用 RTS 要求我们停发(telemetry_pump 会因此暂停)。
   持续为 1 且 QLEN 涨满 = 模块没接好或 PC 侧没在收 */
uint8_t telemetry_rts_blocked(void)
{
    return gpio_get_level(WIRELESS_UART_RTS_PIN) ? 1u : 0u;
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
                 保留这一列的意义是让日志自带"当时命令的是多少速度"
           nr/fr: 两段前瞻近段/远段实际投票的行数。fr=0 → 远段整段无信息,
                 误差退化成纯近段、前瞻当帧失效。这是唯一的降级路径(R6)
           farw: 当前 Far W%,与 kp/kd 一样是"让日志自带配置"的列 */
        queue_push_line(
            "# t_ms,err,hold,srv,dty,tgt,row,nr,fr,lst,th,crs,kp,kd,farw\r\n");
        s_need_header = 0u;
    }

    if ((frame % TELEM_DIV) != 0u)
    {
        return;
    }

    len = snprintf(buf, sizeof(buf),
                   "%lu,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%u\r\n",
                   (unsigned long)t_ms,
                   (int)out->error_used,
                   (unsigned)ti->err_hold,
                   (unsigned)out->servo_pwm,
                   (unsigned)out->duty,
                   (unsigned)out->duty_target,
                   (unsigned)ti->valid_rows,
                   (unsigned)ti->near_rows,
                   (unsigned)ti->far_rows,
                   (unsigned)ti->both_lost_rows,
                   (unsigned)ti->threshold,
                   (unsigned)ti->cross_valid,
                   (double)steer_kp,
                   (double)steer_kd,
                   (unsigned)steer_far_w_pct);
    if (len > 0)
    {
        uint16_t send_len = (len < (int)sizeof(buf))
                          ? (uint16_t)len
                          : (uint16_t)(sizeof(buf) - 1u);
        (void)queue_push((const uint8_t *)buf, send_len);
    }
}
