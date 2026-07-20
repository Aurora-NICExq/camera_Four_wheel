/*********************************************************************************************************************
 * 模块：image.c — 图像流水线（纯逻辑层，可在 PC 上用 gcc 编译）。
 * 阈值(大津/手动) → 八邻域双边跟踪 → 加权中线误差。逐像素路径全整数，无动态分配、无跨帧状态。
 * 行坐标：对外 row=0 是图像最底行（离车最近）；相机原始数组第 0 行在顶部，内部用 RAW_ROW() 翻转。
 ********************************************************************************************************************/
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "hybrid_8n_longest_col/hybrid_track.h"

/* 菜单可调二值化阈值（菜单直接编辑此 volatile 全局）：
 *   image_threshold == 0 → 自动大津法（上电默认，行为与精简前一致）；
 *   image_threshold >  0 → 手动固定阈值。
 * PC replay 不编译菜单，恒为 0 → 回放走大津法，逐位不变。 */
volatile int16_t image_threshold = 0;

/* 原始图像行号换算：逻辑行(近=0) → 相机原始行(顶=0) */
#define RAW_ROW(row)    ((uint8_t)(IMG_H - 1u - (row)))

/* 二值判定：≥ 阈值为白（赛道），< 阈值为黑（背景/边界） */
#define IS_WHITE(px, th)    ((px) >= (th))

/*===================================================================================================================
 * 一、二值化阈值：快速直方图大津法（整数实现）
 *==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * otsu_threshold — 抽样直方图 + 大津法求最优阈值
 * 输入：img 原始灰度图
 * 返回：二值化阈值 [OTSU_THRESHOLD_MIN, OTSU_THRESHOLD_MAX]
 * 说明：行/列按 OTSU_ROW_STEP/OTSU_COL_STEP 抽样 —— 阈值是全局统计量，抽样 1/4 像素
 *       对结果几乎无影响，却省下大半耗时。全程整数：方差比较采用
 *       w0*w1*(mu0-mu1)^2 形式，用 64 位避免溢出（65535 像素 × 255 灰度平方级别）。
 *------------------------------------------------------------------------------------------------------------------*/
static uint8_t otsu_threshold(const uint8_t img[IMG_H][IMG_W])
{
    uint32_t hist[256] = {0};
    uint32_t total = 0;
    uint16_t r, c;
    uint16_t i;

    for (r = 0; r < IMG_H; r += OTSU_ROW_STEP)
    {
        for (c = 0; c < IMG_W; c += OTSU_COL_STEP)
        {
            hist[img[r][c]]++;
            total++;
        }
    }

    /* 全图灰度加权和 */
    uint64_t sum_all = 0;
    for (i = 0; i < 256; i++)
    {
        sum_all += (uint64_t)i * hist[i];
    }

    uint64_t best_var = 0;      /* 最大类间方差（未除以 total^2 的等价形式） */
    uint16_t best_th  = FIXED_THRESHOLD;
    uint32_t w0 = 0;            /* 前景（黑侧）像素数 */
    uint64_t sum0 = 0;          /* 前景灰度和 */

    for (i = 0; i < 256; i++)
    {
        w0 += hist[i];
        if (w0 == 0)
        {
            continue;
        }
        uint32_t w1 = total - w0;
        if (w1 == 0)
        {
            break;
        }
        sum0 += (uint64_t)i * hist[i];

        /* mu0 = sum0/w0, mu1 = (sum_all-sum0)/w1
         * 类间方差 ∝ w0*w1*(mu0-mu1)^2；为保持整数，比较 (sum0*w1 - (sum_all-sum0)*w0)^2 / (w0*w1) */
        int64_t diff = (int64_t)(sum0 * w1) - (int64_t)((sum_all - sum0) * w0);
        uint64_t d2 = (uint64_t)((diff < 0) ? -diff : diff);
        /* 先除后乘防溢出：d2 / w0 / w1 * d2 会丢精度，改为 (d2/w0) * (d2/w1) 的折中。
         * 精度损失对 8 位阈值的影响 < 1 灰度级，可接受。 */
        uint64_t var = (d2 / w0) * (d2 / w1);
        if (var > best_var)
        {
            best_var = var;
            best_th  = i;
        }
    }

    if (best_th < OTSU_THRESHOLD_MIN)
    {
        best_th = OTSU_THRESHOLD_MIN;   /* 画面几乎全黑时大津会漂到过低阈值，钳制兜底 */
    }
    if (best_th > OTSU_THRESHOLD_MAX)
    {
        best_th = OTSU_THRESHOLD_MAX;
    }
    return (uint8_t)best_th;
}


/*===================================================================================================================
 * 四、加权转向误差（双权重表按占空比混合）
 *==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * weighted_error — 误差 = Σ w(row)·(mid−中心) / Σ w(row)
 * 输入：duty_now ∈ [0,10000]，混合系数 k = duty/DUTY_HARD_CAP（≥1 按 1 算），Q8 定点。
 * 为什么前瞻必须随速度增长：本车减速只能滑行，速度越高滑行距离越长，
 * "发现弯道 → 车速降到能过弯"所需的提前量随速度平方级增长；
 * 低速表近重（近端像素分辨率高、噪声小，稳定压线），高速表远移（提前看弯提前收油打舵）。
 * 行带权重再按行置信缩放：双边实测行全权重，单边重建行 STEER_W_SINGLE_EDGE_PCT，
 * 双边丢失预测行 STEER_W_BOTH_LOST_PCT——预测中线本就是下方实测行推出来的，
 * 再计入等于重复投票，宽白区里还会把假想走廊当真边线拉舵。
 *------------------------------------------------------------------------------------------------------------------*/
static int16_t weighted_error(const track_info_t *ti, uint16_t duty_now)
{
    static const uint8_t w_low[STEER_W_BANDS]  = STEER_WEIGHTS_LOWSPEED;
    static const uint8_t w_high[STEER_W_BANDS] = STEER_WEIGHTS_HIGHSPEED;

    /* 混合系数 k ∈ [0,256]，Q8：k=0 全用低速表，k=256 全用高速表 */
    uint32_t k = ((uint32_t)duty_now * 256u) / DUTY_HARD_CAP;
    if (k > 256u)
    {
        k = 256u;
    }

    int32_t acc   = 0;
    int32_t w_sum = 0;
    uint8_t r;

    for (r = 0; r < ti->valid_rows; r++)
    {
        uint8_t band = (uint8_t)(r / STEER_W_BAND_ROWS);
        if (band >= STEER_W_BANDS)
        {
            band = STEER_W_BANDS - 1;
        }
        /* Q8 混合：w = w_low·(256−k) + w_high·k（保持整数，最后统一归一化） */
        int32_t w = (int32_t)w_low[band] * (int32_t)(256u - k)
                  + (int32_t)w_high[band] * (int32_t)k;
        if (ti->left_lost[r] && ti->right_lost[r])
        {
            w = (w * STEER_W_BOTH_LOST_PCT) / 100;
        }
        else if (ti->left_lost[r] || ti->right_lost[r])
        {
            w = (w * STEER_W_SINGLE_EDGE_PCT) / 100;
        }
        acc   += w * ((int16_t)ti->mid[r] - IMG_CENTER);
        w_sum += w;
    }

    if (w_sum == 0)
    {
        return 0;       /* 没有可信行：误差记 0，失控保护（行数/丢线比例判据）会接管 */
    }
    return (int16_t)(acc / w_sum);
}

/*-------------------------------------------------------------------------------------------------------------------
 * image_track_invalid — 控制输出前的纯逻辑图像健康判定
 * valid_rows 低覆盖全黑/冲出；双边同时丢失比例高覆盖全白/严重过曝。
 *------------------------------------------------------------------------------------------------------------------*/
uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe)
{
    uint16_t lost_pct_lhs;
    uint16_t rows_rhs;

    *severe = 0;
    if (ti->valid_rows < FAILSAFE_MIN_ROWS)
    {
        *severe = (uint8_t)(ti->valid_rows == 0u);
        return 1;
    }

    lost_pct_lhs = (uint16_t)ti->both_lost_rows * 100u;
    rows_rhs = (uint16_t)ti->valid_rows;
    if (lost_pct_lhs >= rows_rhs * FAILSAFE_SEVERE_BOTH_LOST_PCT)
    {
        *severe = 1;
        return 1;
    }
    return (uint8_t)(lost_pct_lhs >= rows_rhs * FAILSAFE_MAX_BOTH_LOST_PCT);
}

/*-------------------------------------------------------------------------------------------------------------------
 * image_process — 流水线总入口。执行顺序即数据依赖顺序。
 *------------------------------------------------------------------------------------------------------------------*/
void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out)
{
    /* 1. 阈值：image_threshold>0 用手动固定值，=0 用自动大津法 */
    out->threshold = (image_threshold > 0) ? (uint8_t)image_threshold
                                           : otsu_threshold(img);

    /* 2. 赛道几何：最长白列起种 + 八邻域双边跟踪 */
    hybrid_track_extract(img, out->threshold, out, (hybrid_track_diag_t *)0);

    /* 3. 加权中线误差（低/高速权重按占空比混合）—— 唯一的控制输出 */
    out->error = weighted_error(out, duty_now);
}
