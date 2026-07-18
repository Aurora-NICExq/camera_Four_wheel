/*********************************************************************************************************************
 * 模块：image.c — 图像流水线与元素检测器（纯逻辑层，可在 PC 上用 gcc 编译）
 *
 * 依赖：只有 <stdint.h> 与 config.h/image.h/perf.h —— 不含任何 MCU / 逐飞头文件
 *       （perf.h 亦为纯逻辑层：PERF_PROFILE=0 时全部挂钩展开为空，见 perf.h 说明）。
 * 规则：逐像素路径只用整数运算（浮点在 TC264 上不慢，但整数保证 PC/MCU 行为逐位一致，
 *       且养成习惯避免把浮点误带进 DMA 中断竞争的热路径）；无动态分配；无跨帧静态状态
 *       （同一帧输入必然产生同一输出 —— replay 回放与车载逐位一致的前提）。
 *
 * 行坐标约定：本文件对外的 row=0 是图像最底行（离车最近）。摄像头原始数组第 0 行是画面顶部，
 *             所以内部一律用 RAW_ROW(row) 翻转访问。
 ********************************************************************************************************************/
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "perf.h"
#if USE_HYBRID_TRACKING
#include "hybrid_8n_longest_col/hybrid_track.h"
#endif

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

#if !USE_HYBRID_TRACKING
/*===================================================================================================================
 * 二、最长白列（最长白列 = 主自由方向，长度 = 视觉前瞻距离）
 *==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * longest_white_column — 每列从最底行向上数连续白像素，取最长者
 * 输出：*col 最长白列列号，返回其连续白行数（= 有效行数）
 * 说明：从"车头正前方"（图像底部）出发的连续白列就是车能直接开过去的自由空间；
 *       它天然融合了"赛道去哪了"（列号）与"能看多远"（长度）两个信息。
 *       COL_STEP 抽样是耗时超预算时的第一档降级手段。
 *------------------------------------------------------------------------------------------------------------------*/
static uint8_t longest_white_column(const uint8_t img[IMG_H][IMG_W], uint8_t th, uint8_t *col)
{
    uint8_t  best_len = 0;
    uint8_t  best_col = IMG_CENTER;
    uint16_t c;

    for (c = 0; c < IMG_W; c += COL_STEP)
    {
        uint8_t len = 0;
        uint8_t r;
        for (r = 0; r < IMG_H; r++)
        {
            if (!IS_WHITE(img[RAW_ROW(r)][c], th))
            {
                break;                      /* 遇黑即止：要的是"连续"白，不是白像素总数 */
            }
            len++;
        }
        if (len > best_len)
        {
            best_len = len;
            best_col = (uint8_t)c;
        }
    }

    *col = best_col;
    return best_len;
}

/*===================================================================================================================
 * 三、逐行边线搜索 + 丢线重建
 *==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * search_edges — 从最长白列向左右扫描白→黑边界，逐行填 left/right/lost 标志
 * 说明：以最长白列为种子向两侧找边，比"从图像左右边缘向内找"抗干扰：
 *       种子必然在赛道内，扫到的第一个白→黑跳变就是真边线；
 *       从图像边缘找则会先撞上赛道外的各种黑色杂物。
 *       扫到图像边缘还没变黑 → 记该侧丢线（十字/环口/大弯都会出现）。
 *------------------------------------------------------------------------------------------------------------------*/
static void search_edges(const uint8_t img[IMG_H][IMG_W], uint8_t th, track_info_t *ti)
{
    uint8_t r;

    for (r = 0; r < ti->valid_rows; r++)
    {
        const uint8_t *raw = img[RAW_ROW(r)];
        int16_t c;

        /* 左边线：从种子列向左找白→黑跳变 */
        ti->left_lost[r] = 1;
        ti->left[r]      = 0;
        for (c = ti->longest_col; c > 0; c--)
        {
            if (!IS_WHITE(raw[c - 1], th))
            {
                ti->left[r]      = (uint8_t)c;
                ti->left_lost[r] = 0;
                break;
            }
        }

        /* 右边线：向右找白→黑跳变 */
        ti->right_lost[r] = 1;
        ti->right[r]      = IMG_W - 1;
        for (c = ti->longest_col; c < IMG_W - 1; c++)
        {
            if (!IS_WHITE(raw[c + 1], th))
            {
                ti->right[r]      = (uint8_t)c;
                ti->right_lost[r] = 0;
                break;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * rebuild_lost_edges — 帧内学习赛道宽度表，重建丢失边线，生成中线
 * 步骤：
 *   1. 双边完好的行：width = right-left（若在合法区间），存入宽度表并线性外推填补空档；
 *   2. 单边丢失：丢失边 = 另一边 ± 该行宽度表值；
 *   3. 双边丢失：中线沿用下方最近完好行的中线（向上外推 —— 十字里就是"照直冲"）；
 *   4. mid = (left+right)/2，逐行钳制在图像范围内。
 * 为什么宽度表按行学习：透视投影下赛道宽度随行号非线性变化，一条全局宽度会在远端重建出
 * 系统性偏差；用"本帧双边行实测 + 邻行插值"最贴近当下透视关系。
 *------------------------------------------------------------------------------------------------------------------*/
static void rebuild_lost_edges(track_info_t *ti)
{
    uint8_t r;
    ti->both_lost_rows = 0;

    /* --- 第 1 步：实测宽度 + 向上就近填充（下方最近实测值） --- */
    uint8_t last_width = WIDTH_TABLE_DEFAULT;
    for (r = 0; r < ti->valid_rows; r++)
    {
        if (!ti->left_lost[r] && !ti->right_lost[r])
        {
            int16_t w = (int16_t)ti->right[r] - (int16_t)ti->left[r];
            if (w >= WIDTH_MIN_PX && w <= WIDTH_MAX_PX)
            {
                last_width = (uint8_t)w;
            }
        }
        ti->width[r] = last_width;      /* 丢线行沿用下方最近的实测宽度（透视下相邻行宽度近似） */
    }

    /* --- 第 2/3 步：重建丢失边 --- */
    uint8_t last_mid = IMG_CENTER;      /* 双边全丢时的外推基准：下方最近可信中线 */
    for (r = 0; r < ti->valid_rows; r++)
    {
        uint8_t lost_l = ti->left_lost[r];
        uint8_t lost_r = ti->right_lost[r];

        if (lost_l && lost_r)
        {
            /* 双边丢失：无任何本行信息，中线外推 = 下方最近可信行的中线。
             * 十字中央就是这种情况 —— 外推即"保持直行冲过去"，与十字策略一致。 */
            ti->both_lost_rows++;
            int16_t half = (int16_t)(ti->width[r] / 2u);
            int16_t l = (int16_t)last_mid - half;
            int16_t rr = (int16_t)last_mid + half;
            ti->left[r]  = (uint8_t)((l  < 0) ? 0 : ((l  > IMG_W - 1) ? (IMG_W - 1) : l));
            ti->right[r] = (uint8_t)((rr < 0) ? 0 : ((rr > IMG_W - 1) ? (IMG_W - 1) : rr));
        }
        else if (lost_l)
        {
            /* 只丢左边：左 = 右 − 宽 */
            int16_t l = (int16_t)ti->right[r] - (int16_t)ti->width[r];
            ti->left[r] = (uint8_t)((l < 0) ? 0 : l);
        }
        else if (lost_r)
        {
            /* 只丢右边：右 = 左 + 宽 */
            int16_t rr = (int16_t)ti->left[r] + (int16_t)ti->width[r];
            ti->right[r] = (uint8_t)((rr > IMG_W - 1) ? (IMG_W - 1) : rr);
        }

        ti->mid[r] = (uint8_t)(((uint16_t)ti->left[r] + (uint16_t)ti->right[r]) / 2u);

        if (!lost_l || !lost_r)
        {
            last_mid = ti->mid[r];      /* 至少一边实测的行才更新外推基准 */
        }
    }
}
#endif /* !USE_HYBRID_TRACKING */

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
 * segment_slope_q8 — 行带 [lo,hi] 内中线斜率（Q8 定点：Δmid×256/Δrow）
 * 只取带内首尾两点求割线斜率：比逐点最小二乘便宜得多，对"远近段弯直差异"这种粗特征足够。
 * 端点只允许落在"至少一边实测"的行上（预测行的中线不是测量值）；带内可信端点不足
 * CURV_MIN_SPAN_ROWS 行距时置 *ok=0——调用方据此把本帧曲率记 0，交给行数减速表兜底。
 *------------------------------------------------------------------------------------------------------------------*/
static int16_t segment_slope_q8(const track_info_t *ti, uint8_t lo, uint8_t hi, uint8_t *ok)
{
    *ok = 0;
    if (hi >= ti->valid_rows)
    {
        hi = (uint8_t)(ti->valid_rows - 1u);
    }
    while (lo < hi && ti->left_lost[lo] && ti->right_lost[lo])
    {
        lo++;
    }
    while (hi > lo && ti->left_lost[hi] && ti->right_lost[hi])
    {
        hi--;
    }
    if (hi <= lo || (uint8_t)(hi - lo) < CURV_MIN_SPAN_ROWS ||
        (ti->left_lost[lo] && ti->right_lost[lo]))
    {
        return 0;
    }
    *ok = 1;
    int32_t dm = (int32_t)ti->mid[hi] - (int32_t)ti->mid[lo];
    return (int16_t)((dm * 256) / (int32_t)(hi - lo));
}

/*===================================================================================================================
 * 五、元素检测器 —— 纯函数，单帧判定，不去抖（去抖是 fsm.c 的结构性职责）
 *==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * detect_cross — 十字特征：行带内一段双边丢失，且丢失带下方与上方都有双边完好行
 * "上下都有完好行"是十字与冲出赛道的本质区别：十字对面的赛道仍然可见。
 *------------------------------------------------------------------------------------------------------------------*/
uint8_t detect_cross(const track_info_t *ti)
{
#if ENABLE_CROSS
    uint8_t start;
    uint8_t hi = (ti->valid_rows < CROSS_BAND_ROW_HI) ? ti->valid_rows : CROSS_BAND_ROW_HI;
    for (start = CROSS_BAND_ROW_LO; start < hi; start++)
    {
        uint8_t end;
        uint8_t good_below = 0;
        uint8_t good_above = 0;
        int16_t r;

        if (!(ti->left_lost[start] && ti->right_lost[start]))
        {
            continue;
        }

        end = start;
        while (end < hi && ti->left_lost[end] && ti->right_lost[end])
        {
            end++;
        }

        for (r = (int16_t)start - 1; r >= CROSS_BAND_ROW_LO; r--)
        {
            if (ti->left_lost[r] || ti->right_lost[r])
            {
                break;
            }
            good_below++;
        }
        for (r = end; r < hi; r++)
        {
            if (ti->left_lost[r] || ti->right_lost[r])
            {
                break;
            }
            good_above++;
        }

        if ((uint8_t)(end - start) >= CROSS_MIN_BOTH_LOST &&
            good_below >= CROSS_MIN_GOOD_BELOW &&
            good_above >= CROSS_MIN_GOOD_ABOVE)
        {
            return 1;
        }

        /* 本段已完整检查；外层直接跳到下一段，避免把同一缺口重复扫描。 */
        if (end > start)
        {
            start = (uint8_t)(end - 1u);
        }
    }
    return 0;
#else
    (void)ti;
    return 0;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * ring_side_signature — 环岛单侧特征（左右共用的内部实现）
 * 特征：弧侧（环口所在侧）行带内出现"完好→丢失(缺口)→再完好"的三段结构，
 *       且对侧（连续侧）几乎不丢线。"缺口上方边线重现"区分环岛与普通大弯的单边丢失。
 * 输出：命中返回 1，*gap_lo 回填缺口下沿行号（入环时机判断用）。
 *------------------------------------------------------------------------------------------------------------------*/
static uint8_t ring_side_signature(const track_info_t *ti,
                                   const uint8_t *arc_lost, const uint8_t *solid_lost,
                                   uint8_t *gap_lo)
{
    uint8_t r;
    uint8_t hi = (ti->valid_rows < RING_BAND_ROW_HI) ? ti->valid_rows : RING_BAND_ROW_HI;

    /* 连续侧必须真的连续 */
    uint8_t solid_lost_cnt = 0;
    for (r = RING_BAND_ROW_LO; r < hi; r++)
    {
        if (solid_lost[r])
        {
            solid_lost_cnt++;
        }
    }
    if (solid_lost_cnt > RING_SOLID_MAX_LOST)
    {
        return 0;
    }

    /* 弧侧：严格寻找一段连续的 完好→缺口→完好 结构。
     * 旧实现会把多个零散缺口累计，容易让噪声凑够阈值。 */
    for (r = RING_BAND_ROW_LO; r < hi; r++)
    {
        uint8_t end;
        uint8_t good_below = 0;
        uint8_t good_above = 0;
        int16_t rr;

        if (!arc_lost[r])
        {
            continue;
        }

        end = r;
        while (end < hi && arc_lost[end])
        {
            end++;
        }
        for (rr = (int16_t)r - 1; rr >= RING_BAND_ROW_LO; rr--)
        {
            if (arc_lost[rr])
            {
                break;
            }
            good_below++;
        }
        for (rr = end; rr < hi; rr++)
        {
            if (arc_lost[rr])
            {
                break;
            }
            good_above++;
        }

        if ((uint8_t)(end - r) >= RING_ARC_MIN_LOST &&
            good_below >= RING_ARC_MIN_GOOD_BELOW &&
            good_above >= RING_ARC_MIN_GOOD_ABOVE)
        {
            *gap_lo = r;
            return 1;
        }

        if (end > r)
        {
            r = (uint8_t)(end - 1u);
        }
    }
    return 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * detect_ring_left / detect_ring_right — 左/右环岛（互为镜像）
 * 左环岛：环在赛道左侧 → 左边线出现环口缺口（弧侧=左），右边线连续（连续侧=右）。
 *------------------------------------------------------------------------------------------------------------------*/
uint8_t detect_ring_left(const track_info_t *ti)
{
#if ENABLE_RING
    uint8_t gap_lo;
    return ring_side_signature(ti, ti->left_lost, ti->right_lost, &gap_lo);
#else
    (void)ti;
    return 0;
#endif
}

uint8_t detect_ring_right(const track_info_t *ti)
{
#if ENABLE_RING
    uint8_t gap_lo;
    return ring_side_signature(ti, ti->right_lost, ti->left_lost, &gap_lo);
#else
    (void)ti;
    return 0;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * detect_ramp — 坡道（占位实现）：远行带实测宽度显著大于近处学习值（上坡视角变宽）
 * TODO：本赛季规则确定后按实车视角完善（备选特征：地平线行号跳变）。默认 ENABLE_RAMP=0。
 *------------------------------------------------------------------------------------------------------------------*/
uint8_t detect_ramp(const track_info_t *ti)
{
#if ENABLE_RAMP
    uint8_t r;
    uint8_t wide_rows = 0;
    uint8_t hi = (ti->valid_rows < RAMP_BAND_ROW_HI) ? ti->valid_rows : RAMP_BAND_ROW_HI;

    for (r = RAMP_BAND_ROW_LO; r < hi; r++)
    {
        if (!ti->left_lost[r] && !ti->right_lost[r])
        {
            uint16_t measured = (uint16_t)(ti->right[r] - ti->left[r]);
            if (measured * 100u > (uint16_t)ti->width[r] * RAMP_WIDTH_NUM)
            {
                wide_rows++;
            }
        }
    }
    return (uint8_t)(wide_rows >= RAMP_MIN_ROWS);
#else
    (void)ti;
    return 0;
#endif
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

/*===================================================================================================================
 * 六、流水线总入口
 *==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * image_process — 见 image.h 的接口说明。执行顺序即数据依赖顺序。
 *------------------------------------------------------------------------------------------------------------------*/
void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out)
{
    /* 1. 阈值 */
    PERF_BEGIN(PF_OTSU);
#if USE_FIXED_THRESHOLD
    out->threshold = FIXED_THRESHOLD;
#else
    out->threshold = otsu_threshold(img);
#endif
    PERF_END(PF_OTSU);

    /* 2/3. 赛道几何：融合模式由最长白列起种，再用八邻域双边跟踪；关闭开关时保留原算法。 */
#if USE_HYBRID_TRACKING
    PERF_BEGIN(PF_EDGES);
    hybrid_track_extract(img, out->threshold, out, (hybrid_track_diag_t *)0);
    PERF_END(PF_EDGES);
#else
    PERF_BEGIN(PF_LONGCOL);
    out->valid_rows = longest_white_column(img, out->threshold, &out->longest_col);
    PERF_END(PF_LONGCOL);

    PERF_BEGIN(PF_EDGES);
    search_edges(img, out->threshold, out);
    PERF_END(PF_EDGES);
    PERF_BEGIN(PF_REBUILD);
    rebuild_lost_edges(out);
    PERF_END(PF_REBUILD);
#endif

    /* 4. 加权转向误差（低/高速权重按当前占空比混合） */
    PERF_BEGIN(PF_ERRCURV);
    out->error = weighted_error(out, duty_now);

    /* 5. 曲率：远段斜率 − 近段斜率。有效行覆盖不到远段、或任一段缺少可信实测端点时
     *    记 0（行数减速表会兜底降速；且低置信帧禁止出弯/boost 再加速，见 control.c）。 */
    if (out->valid_rows > CURV_FAR_ROW_LO)
    {
        uint8_t near_ok;
        uint8_t far_ok;
        int16_t s_near = segment_slope_q8(out, CURV_NEAR_ROW_LO, CURV_NEAR_ROW_HI, &near_ok);
        int16_t s_far  = segment_slope_q8(out, CURV_FAR_ROW_LO,  CURV_FAR_ROW_HI,  &far_ok);
        out->curvature = (near_ok && far_ok) ? (int16_t)(s_far - s_near) : 0;
    }
    else
    {
        out->curvature = 0;
    }
    PERF_END(PF_ERRCURV);

    /* 6. 元素检测器（单帧原始输出，去抖交给 fsm.c）。 */
    PERF_BEGIN(PF_DET_GEOM);
    out->det_cross      = detect_cross(out);
    out->det_ring_left  = detect_ring_left(out);
    out->det_ring_right = detect_ring_right(out);
    out->det_ramp       = detect_ramp(out);
    PERF_END(PF_DET_GEOM);

    /* 7. 检测特征强度（FSM 转移轨迹记录用）：取"最有戏剧性"的一个 */
    out->det_value   = (int16_t)out->both_lost_rows;
    out->inflect_row = 0xFF;    /* 拐点行号：当前实现未单独导出，保留字段供调试扩展 */
}
