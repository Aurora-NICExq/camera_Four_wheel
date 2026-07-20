/* perf.c - on-car stage timing (PERF_PROFILE) */
#include "config.h"

#if PERF_PROFILE

#include <stdio.h>
#include "zf_common_headfile.h"
#include "motor.h"          /* hal_time_us() */
#include "perf.h"

typedef struct
{
    uint32_t min_us;
    uint32_t max_us;
    uint64_t sum_us;
    uint32_t cnt;
} pf_stat_t;

static const char *const g_stage_name[PF_STAGE_COUNT] =
{
    "OTSU    ", "LONGCOL ", "EDGES   ", "REBUILD ", "ERRCURV ",
    "DET_GEOM", "FSM     ", "CONTROL ", "DISPLAY ", "TOTAL   ",
};

static uint32_t  g_t0     [PF_STAGE_COUNT];     /* 本次 begin 的时间戳                    */
static uint32_t  g_scratch[PF_STAGE_COUNT];     /* 本帧累计（支持一帧内多对 begin/end）   */
static pf_stat_t g_stat   [PF_STAGE_COUNT];
static uint16_t  g_hist   [PF_STAGE_COUNT][PERF_HIST_BINS];     /* p99 用，计数饱和不回绕 */
static uint32_t  g_frames;

void perf_stage_begin(uint8_t stage_id)
{
    g_t0[stage_id] = hal_time_us();
}

void perf_stage_end(uint8_t stage_id)
{
    g_scratch[stage_id] += hal_time_us() - g_t0[stage_id];
}

static void perf_report(void);

void perf_frame_commit(void)
{
    uint8_t s;
    for (s = 0; s < PF_STAGE_COUNT; s++)
    {
        uint32_t us = g_scratch[s];
        g_scratch[s] = 0;

        pf_stat_t *st = &g_stat[s];
        if (st->cnt == 0 || us < st->min_us)
        {
            st->min_us = us;
        }
        if (us > st->max_us)
        {
            st->max_us = us;
        }
        st->sum_us += us;
        st->cnt++;

        uint32_t bin = us / PERF_HIST_BIN_US;
        if (bin >= PERF_HIST_BINS)
        {
            bin = PERF_HIST_BINS - 1u;
        }
        if (g_hist[s][bin] < 0xFFFFu)
        {
            g_hist[s][bin]++;
        }
    }

    g_frames++;
    if ((g_frames % PERF_REPORT_DIV) == 0u)
    {
        perf_report();
    }
}

static void perf_report(void)
{
    char buf[96];
    int  len;
    uint8_t s;

    len = snprintf(buf, sizeof(buf), "#PERF n=%lu fps=%u budget_us=%lu\r\n",
                   (unsigned long)g_frames, (unsigned)FRAMES_PER_SECOND,
                   (unsigned long)(1000000uL / FRAMES_PER_SECOND));
    if (len > 0)
    {
        wireless_uart_send_buffer((const uint8 *)buf, (uint32)len);
    }

    for (s = 0; s < PF_STAGE_COUNT; s++)
    {
        const pf_stat_t *st = &g_stat[s];
        if (st->cnt == 0)
        {
            continue;
        }

        uint32_t target = st->cnt - st->cnt / 100u;     /* 99% 位置的帧序号 */
        uint32_t acc = 0;
        uint32_t p99_us = (uint32_t)PERF_HIST_BINS * PERF_HIST_BIN_US;
        uint16_t b;
        for (b = 0; b < PERF_HIST_BINS; b++)
        {
            acc += g_hist[s][b];
            if (acc >= target)
            {
                p99_us = ((uint32_t)b + 1u) * PERF_HIST_BIN_US;
                break;
            }
        }

        len = snprintf(buf, sizeof(buf), "#PERF %s min=%lu mean=%lu max=%lu p99=%lu\r\n",
                       g_stage_name[s],
                       (unsigned long)st->min_us,
                       (unsigned long)(st->sum_us / st->cnt),
                       (unsigned long)st->max_us,
                       (unsigned long)p99_us);
        if (len > 0)
        {
            wireless_uart_send_buffer((const uint8 *)buf, (uint32)len);
        }
    }
}

#else

typedef int perf_c_empty_translation_unit;      /* ISO C 不允许空翻译单元 */

#endif /* PERF_PROFILE */
