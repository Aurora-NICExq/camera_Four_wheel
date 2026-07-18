/*********************************************************************************************************************
 * 模块：perf.c — 车载分段耗时统计（PERF_PROFILE=1 时才编译出代码）
 *
 * 输出（每 PERF_REPORT_DIV 帧经无线串口发一组，PASS 1 基线表的数据源）：
 *   #PERF n=<帧数> fps=50 budget_us=20000
 *   #PERF <阶段名> min=<µs> mean=<µs> max=<µs> p99=<µs>   （每阶段一行）
 *
 * p99 用直方图求：桶宽 PERF_HIST_BIN_US，桶数 PERF_HIST_BINS，超量程并入末桶
 * （p99 显示为量程上限，同时 max 单独精确记录，不受桶量程限制）。
 *
 * 注意：报表发送本身约需十几 ms（>1 帧周期），只发生在报表帧 —— 该帧可能丢 1 帧相机数据。
 *       剖析模式本来就配合 DEBUG_NO_DRIVE=1 静态使用；统计值在发送前已结算，不被污染。
 *       统计自上电累计，重新解锁不清零 —— 跑一段有代表性的路线后读一次报表即可。
 ********************************************************************************************************************/
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

/*-------------------------------------------------------------------------------------------------------------------
 * perf_report — 逐阶段一行报表发无线串口。p99 = 直方图累计计数首次达到 99% 处的桶上沿。
 *------------------------------------------------------------------------------------------------------------------*/
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
