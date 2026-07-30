/* telemetry.c - 逐飞无线串口 CSV 遥测(有界队列,不阻塞主循环) */
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
static uint8_t  s_wireless_ok;
static uint32_t s_tx_bytes;

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

void telemetry_reinit(void)
{
    uint8_t st;

    st = wireless_uart_init();
    /* 逐飞库:0=成功,1=自动波特率失败(RTS 未接/模块版本低于 V2.0/接线错) */
    s_wireless_ok = (st == 0u) ? 1u : 0u;

    /* 【发不出数据的根因,已对照库源码确认】
       库默认 WIRELESS_UART_AUTO_BAUD_RATE=1。握手期间 wireless_uart_init() 会把
       RTS 从输入改成【推挽输出】,用电平时序告诉模块进入自动波特率模式:
           gpio_init(RTS, GPO, rts_init_status, GPO_PUSH_PULL);
       而把 RTS 恢复成输入的那句
           gpio_init(RTS, GPI, 0, GPI_PULL_UP);
       只存在于 do{}while(0) 的【成功路径末尾】。握手失败时是
           return_state = 1; break;
       直接跳出,于是 RTS 永久停留在推挽输出状态。
       此后 telemetry_pump() 里的 gpio_get_level(RTS) 读到的是【自己的输出电平】,
       恒为高 → 每次调用立刻 break → 一个字节都发不出去,队列涨满后全丢。
       这里无条件恢复成上拉输入:既让流控读到模块的真实电平,
       也避免模块同时驱动该引脚时的推挽对顶。 */
    gpio_init(WIRELESS_UART_RTS_PIN, GPI, 0, GPI_PULL_UP);

    s_head        = 0u;
    s_tail        = 0u;
    s_count       = 0u;
    s_tx_bytes    = 0u;
    s_need_header = 1u;
}

void telemetry_init(void)
{
    telemetry_reinit();
    s_frame       = 0u;
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

        /* 只有握手成功时 RTS 才是模块真实驱动的流控输入,才值得遵守。
           握手失败说明模块版本低于 V2.0(不支持自动波特率)或 RTS 没接线,
           此时 RTS 的读数没有物理意义,一味遵守它等于永久静默。
           库头注释写明 115200 就是模块的出厂默认波特率,直接发即可。 */
        if (s_wireless_ok && gpio_get_level(WIRELESS_UART_RTS_PIN))
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
        /* 与 wireless_uart_send_buffer 内部一致:RTS 低才写 UART。
         * 不用 send_buffer 本体——它会 system_delay_ms 阻塞主循环 */
        uart_write_buffer(WIRELESS_UART_INDEX, chunk, n);
        s_tx_bytes += (uint32_t)n;
        budget = (uint16_t)(budget - n);
    }
}

uint8_t telemetry_wireless_ok(void)
{
    return s_wireless_ok;
}

uint32_t telemetry_tx_bytes(void)
{
    return s_tx_bytes;
}

/* 逐飞库实际配置的波特率(zf_device_wireless_uart.h,当前 115200)。
   本工程不再试图覆盖它:串口是在 zf_device_wireless_uart.c 里 uart_init 的,
   那个编译单元看不到本工程任何头文件。PC 助手按 UART Test 页显示的这个值设。 */
uint32_t telemetry_baud(void)
{
    return (uint32_t)WIRELESS_UART_BUAD_RATE;
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
                        (unsigned long)WIRELESS_UART_BUAD_RATE);

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
           farw: 当前 Far W%,与 kp/kd 一样是"让日志自带配置"的列
           kp100/kd100: Kp、Kd 的 100 倍整数值(229 = 2.29)。整行不含浮点格式:
                 ADS/Tasking 的 C 库若没链浮点 printf,snprintf 遇 %f 会返回负值,
                 下面 if(len>0) 不成立 → 每一行数据被静默丢弃、只有表头发得出去 */
        if (queue_push_line(
                "# t_ms,err,hold,srv,dty,tgt,row,nr,fr,lst,th,crs,"
                "kp100,kd100,farw\r\n"))
        {
            /* 入队失败(队列正堵着)就不要清标志,否则表头永久丢失、CSV 没法解析 */
            s_need_header = 0u;
        }
    }

    if ((frame % TELEM_DIV) != 0u)
    {
        return;
    }

    len = snprintf(buf, sizeof(buf),
                   "%lu,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
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
                   (unsigned)(uint32_t)(steer_kp * 100.0f + 0.5f),
                   (unsigned)(uint32_t)(steer_kd * 100.0f + 0.5f),
                   (unsigned)steer_far_w_pct);
    if (len > 0)
    {
        uint16_t send_len = (len < (int)sizeof(buf))
                          ? (uint16_t)len
                          : (uint16_t)(sizeof(buf) - 1u);
        (void)queue_push((const uint8_t *)buf, send_len);
    }
}
