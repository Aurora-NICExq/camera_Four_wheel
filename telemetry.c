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

    s_wireless_ok = (st == 0u) ? 1u : 0u;

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

uint32_t telemetry_baud(void)
{
    return (uint32_t)WIRELESS_UART_BUAD_RATE;
}

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
        if (queue_push_line(
                "# t_ms,err,hold,look,far,srv,dty,tgt,lst,th,crs,f2l,ffl,ffr,"
                "kp100,kd100\r\n"))
        {
            s_need_header = 0u;
        }
    }

    if ((frame % TELEM_DIV) != 0u)
    {
        return;
    }

    len = snprintf(buf, sizeof(buf),
                   "%lu,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   (unsigned long)t_ms,
                   (int)out->error_used,
                   (unsigned)ti->err_hold,
                   (unsigned)ti->look_rows,
                   (unsigned)steer_look_far,
                   (unsigned)out->servo_pwm,
                   (unsigned)out->duty,
                   (unsigned)out->duty_target,
                   (unsigned)ti->both_lost_rows,
                   (unsigned)ti->threshold,
                   (unsigned)ti->cross_valid,
                   (unsigned)image_fill_to_look,
                   (unsigned)ti->fill_from_l,
                   (unsigned)ti->fill_from_r,
                   (unsigned)(uint32_t)(steer_kp * 100.0f + 0.5f),
                   (unsigned)(uint32_t)(steer_kd * 100.0f + 0.5f));
    if (len > 0)
    {
        uint16_t send_len = (len < (int)sizeof(buf))
                          ? (uint16_t)len
                          : (uint16_t)(sizeof(buf) - 1u);
        (void)queue_push((const uint8_t *)buf, send_len);
    }
}
