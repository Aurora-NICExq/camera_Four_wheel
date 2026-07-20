/*********************************************************************************************************************
 * 模块：hybrid_track.c — 最长白列锚定 + 八邻域双边跟踪
 *
 * 核心思路：
 *   1. 最长白列只做近端可靠锚点，不再把同一列强行用到所有远端行；
 *   2. 左右边各自优先在上一行边点的 3 邻点（对应二维八邻域中的上、左上、右上）寻找；
 *   3. 严格八邻域断开时，按上一行斜率预测，在小窗口内重捕获；
 *   4. 双边求中线，单边按本帧已学习宽度补线；
 *   5. 双边丢失但种子仍在白区（宽白区/十字中央）：中线跟随种子——每行都是实测白像素，
 *      不再沿两行旧斜率外推几十行假想对角线；这类行标记 lost，控制侧按低置信处理；
 *   6. 断线后重捕获必须几何连续：新边给出的中线偏离走廊超过 HYBRID_REACQUIRE_MAX_JUMP_PX
 *      即拒收，走廊只按 HYBRID_PREDICT_MAX_STEP_PX 每行向候选缓走，禁止一行横跳半幅图；
 *   7. 连续无种子断裂超过 HYBRID_MAX_GAP_ROWS、或锚定白列之外连续双边丢失超过
 *      HYBRID_MAX_PREDICT_ROWS 就停止，避免凭空把整幅图补成赛道。
 *
 * 逐像素路径全整数、无动态内存、无跨帧静态状态，满足原 image.c 的 PC/MCU 一致性约束。
 ********************************************************************************************************************/
#include <stdint.h>
#include "../config.h"
#include "hybrid_track.h"

#define RAW_ROW(row)        ((uint8_t)(IMG_H - 1u - (row)))
#define IS_WHITE(px, th)    ((px) >= (th))

static int16_t clamp_col(int16_t col)
{
    if (col < 0)
    {
        return 0;
    }
    if (col >= IMG_W)
    {
        return (int16_t)(IMG_W - 1);
    }
    return col;
}

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

/* 预测只用于给搜索窗口排序；即使曲率突变，后面的重捕获仍会用真实像素纠正。 */
static int16_t predict_col(int16_t previous, int16_t previous2)
{
    int16_t step = (int16_t)(previous - previous2);
    if (step > HYBRID_MAX_ROW_SHIFT)
    {
        step = HYBRID_MAX_ROW_SHIFT;
    }
    else if (step < -HYBRID_MAX_ROW_SHIFT)
    {
        step = -HYBRID_MAX_ROW_SHIFT;
    }
    return clamp_col((int16_t)(previous + step));
}

/*
 * 最长白列的 tie-break 与旧实现不同：相同长度时选更靠近图像中心的列，避免一整片等长白列时
 * 永远落在最左列。它只负责 row=0 的起种，返回值仍保留为诊断基线。
 */
static uint8_t find_anchor(const uint8_t img[IMG_H][IMG_W], uint8_t th, uint8_t *anchor_col)
{
    uint8_t best_len = 0;
    uint8_t best_col = IMG_CENTER;
    uint16_t c;

    for (c = 0; c < IMG_W; c += COL_STEP)
    {
        uint8_t len = 0;
        uint8_t r;
        for (r = 0; r < IMG_H; r++)
        {
            if (!IS_WHITE(img[RAW_ROW(r)][c], th))
            {
                break;
            }
            len++;
        }
        if (len > best_len ||
            (len == best_len &&
             abs_i16((int16_t)c - IMG_CENTER) < abs_i16((int16_t)best_col - IMG_CENTER)))
        {
            best_len = len;
            best_col = (uint8_t)c;
        }
    }

    *anchor_col = best_col;
    return best_len;
}

static uint8_t is_left_edge(const uint8_t *row, int16_t col, uint8_t th)
{
    return (uint8_t)(col > 0 && IS_WHITE(row[col], th) && !IS_WHITE(row[col - 1], th));
}

static uint8_t is_right_edge(const uint8_t *row, int16_t col, uint8_t th)
{
    return (uint8_t)(col < (IMG_W - 1) && IS_WHITE(row[col], th) && !IS_WHITE(row[col + 1], th));
}

/* 在 [center-radius, center+radius] 内按离 center 由近到远寻找指定类型的边点。 */
static int16_t find_edge_near(const uint8_t *row, uint8_t th, int16_t center,
                              uint8_t radius, uint8_t want_left)
{
    uint8_t d;
    center = clamp_col(center);

    for (d = 0; d <= radius; d++)
    {
        int16_t c0 = (int16_t)(center - d);
        int16_t c1 = (int16_t)(center + d);
        uint8_t hit0 = (uint8_t)(c0 >= 0 && c0 < IMG_W &&
                       (want_left ? is_left_edge(row, c0, th) : is_right_edge(row, c0, th)));
        if (hit0)
        {
            return c0;
        }

        if (d != 0)
        {
            uint8_t hit1 = (uint8_t)(c1 >= 0 && c1 < IMG_W &&
                           (want_left ? is_left_edge(row, c1, th) : is_right_edge(row, c1, th)));
            if (hit1)
            {
                return c1;
            }
        }
    }
    return -1;
}

/*
 * 中线种子优先走上一行中点的上/左上/右上三个像素，这正是单调向上跟踪时需要检查的八邻域子集。
 * 若断开，则围绕斜率预测点做小半径恢复；恢复只找种子，不直接宣布它是边线。
 */
static int16_t find_white_seed(const uint8_t *row, uint8_t th,
                               int16_t previous_mid, int16_t predicted_mid)
{
    uint8_t d;

    /* 严格八邻域：先检查正上，再按预测方向决定两个对角邻点的顺序。 */
    if (IS_WHITE(row[clamp_col(previous_mid)], th))
    {
        return clamp_col(previous_mid);
    }
    if (predicted_mid >= previous_mid)
    {
        if (previous_mid + 1 < IMG_W && IS_WHITE(row[previous_mid + 1], th))
        {
            return (int16_t)(previous_mid + 1);
        }
        if (previous_mid > 0 && IS_WHITE(row[previous_mid - 1], th))
        {
            return (int16_t)(previous_mid - 1);
        }
    }
    else
    {
        if (previous_mid > 0 && IS_WHITE(row[previous_mid - 1], th))
        {
            return (int16_t)(previous_mid - 1);
        }
        if (previous_mid + 1 < IMG_W && IS_WHITE(row[previous_mid + 1], th))
        {
            return (int16_t)(previous_mid + 1);
        }
    }

    for (d = 0; d <= HYBRID_SEED_REACQUIRE_RADIUS; d++)
    {
        int16_t c0 = (int16_t)(predicted_mid - d);
        int16_t c1 = (int16_t)(predicted_mid + d);
        if (c0 >= 0 && c0 < IMG_W && IS_WHITE(row[c0], th))
        {
            return c0;
        }
        if (d != 0 && c1 >= 0 && c1 < IMG_W && IS_WHITE(row[c1], th))
        {
            return c1;
        }
    }
    return -1;
}

/* 以可靠白种子为起点的兜底扫描，与原最长白列法的“从赛道内向两边找第一处黑”一致。 */
static int16_t scan_left_from_seed(const uint8_t *row, uint8_t th, int16_t seed)
{
    int16_t c;
    for (c = seed; c > 0; c--)
    {
        if (!IS_WHITE(row[c - 1], th))
        {
            return c;
        }
    }
    return -1;
}

static int16_t scan_right_from_seed(const uint8_t *row, uint8_t th, int16_t seed)
{
    int16_t c;
    for (c = seed; c < (IMG_W - 1); c++)
    {
        if (!IS_WHITE(row[c + 1], th))
        {
            return c;
        }
    }
    return -1;
}

void hybrid_track_extract(const uint8_t img[IMG_H][IMG_W],
                          uint8_t threshold,
                          track_info_t *out,
                          hybrid_track_diag_t *diag)
{
    uint8_t anchor_len;
    uint8_t anchor_col;
    uint8_t last_width = WIDTH_TABLE_DEFAULT;
    int16_t previous_mid;
    int16_t previous_mid2;
    int16_t previous_left;
    int16_t previous_left2;
    int16_t previous_right;
    int16_t previous_right2;
    uint8_t lost_run = 0;           /* 连续双边丢失（预测）行数，遇实测边归零                     */
    uint8_t noseed_run = 0;         /* 连续既无实测边也无白种子的行数，对应旧 gap_rows 语义       */
    uint8_t last_observed_plus1 = 0;
    uint8_t r;

    anchor_len = find_anchor(img, threshold, &anchor_col);
    out->longest_col = anchor_col;
    out->valid_rows = 0;
    out->both_lost_rows = 0;

    if (diag != (hybrid_track_diag_t *)0)
    {
        diag->longest_rows = anchor_len;
        diag->traced_rows = 0;
        diag->bridged_rows = 0;
        diag->left_reacquires = 0;
        diag->right_reacquires = 0;
    }

    /* 最底行没有任何白像素时不猜赛道；交给原有 FAILSAFE_MIN_ROWS/FAILSAFE_FRAMES 断油。 */
    if (anchor_len == 0)
    {
        return;
    }

    previous_mid = anchor_col;
    previous_mid2 = anchor_col;
    previous_left = anchor_col;
    previous_left2 = anchor_col;
    previous_right = anchor_col;
    previous_right2 = anchor_col;

    for (r = 0; r < IMG_H; r++)
    {
        const uint8_t *row = img[RAW_ROW(r)];
        int16_t predicted_mid = (r < 2u) ? previous_mid : predict_col(previous_mid, previous_mid2);
        int16_t predicted_left = (r < 2u) ? previous_left : predict_col(previous_left, previous_left2);
        int16_t predicted_right = (r < 2u) ? previous_right : predict_col(previous_right, previous_right2);
        int16_t seed;
        int16_t left = -1;
        int16_t right = -1;
        uint8_t left_reacquired = 0;
        uint8_t right_reacquired = 0;
        uint8_t left_lost;
        uint8_t right_lost;
        uint8_t rejected = 0;
        int16_t reject_step = 0;
        int16_t mid;

        if (r == 0u)
        {
            seed = anchor_col;
        }
        else
        {
            seed = find_white_seed(row, threshold, previous_mid, predicted_mid);
        }

        if (r == 0u)
        {
            left = scan_left_from_seed(row, threshold, seed);
            right = scan_right_from_seed(row, threshold, seed);
        }
        else
        {
            /* 第一档严格 ±1，恰好是上一边点朝上一行的三个八邻域位置。 */
            left = find_edge_near(row, threshold, previous_left, 1u, 1u);
            right = find_edge_near(row, threshold, previous_right, 1u, 0u);

            /* 第二档围绕斜率预测点小窗重捕获，应对虚线、噪点和每行横移超过 1 px 的急弯。 */
            if (left < 0)
            {
                left = find_edge_near(row, threshold, predicted_left, HYBRID_EDGE_REACQUIRE_RADIUS, 1u);
                left_reacquired = (uint8_t)(left >= 0);
            }
            if (right < 0)
            {
                right = find_edge_near(row, threshold, predicted_right, HYBRID_EDGE_REACQUIRE_RADIUS, 0u);
                right_reacquired = (uint8_t)(right >= 0);
            }

            /* 第三档沿用最长白列法的强项：种子确定在白区内时，从内向外兜底找边。 */
            if (seed >= 0 && left < 0)
            {
                left = scan_left_from_seed(row, threshold, seed);
                left_reacquired = (uint8_t)(left >= 0);
            }
            if (seed >= 0 && right < 0)
            {
                right = scan_right_from_seed(row, threshold, seed);
                right_reacquired = (uint8_t)(right >= 0);
            }
        }

        /* 左右身份或宽度不合理时，保留更贴近预测的一侧，另一侧交给宽度表补线。 */
        if (left >= 0 && right >= 0)
        {
            int16_t measured_width = (int16_t)(right - left);
            if (measured_width < WIDTH_MIN_PX || measured_width > WIDTH_MAX_PX)
            {
                int16_t left_error = abs_i16((int16_t)(left - predicted_left));
                int16_t right_error = abs_i16((int16_t)(right - predicted_right));
                if (left_error <= right_error)
                {
                    right = -1;
                }
                else
                {
                    left = -1;
                }
            }
        }

        /*
         * 重捕获连续性闸门：断线后的第一批新边（lost_run>0），以及只找到一条边的行，
         * 中线候选必须与走廊（上一行中线）几何连续，否则拒收降级为预测行。
         * 双边同时实测且下方未断线的行免检：两条真实边 + 宽度合法性已是足够强的证据。
         */
        if (r > 0u && (left >= 0 || right >= 0) &&
            (lost_run > 0u || left < 0 || right < 0))
        {
            int16_t candidate_mid;
            if (left >= 0 && right >= 0)
            {
                candidate_mid = (int16_t)((left + right) / 2);
            }
            else if (left >= 0)
            {
                candidate_mid = (int16_t)(left + (int16_t)(last_width / 2u));
            }
            else
            {
                candidate_mid = (int16_t)(right - (int16_t)(last_width / 2u));
            }

            if (abs_i16((int16_t)(candidate_mid - previous_mid)) > HYBRID_REACQUIRE_MAX_JUMP_PX)
            {
                /* 拒收：本行按预测处理，走廊仅向候选缓步收敛。若候选是真赛道，
                 * 几行内走廊就会走到闸门以内重新实测锁定；若是平行伪支路则始终拒之门外。 */
                reject_step = (int16_t)(candidate_mid - previous_mid);
                if (reject_step > HYBRID_PREDICT_MAX_STEP_PX)
                {
                    reject_step = HYBRID_PREDICT_MAX_STEP_PX;
                }
                else if (reject_step < -HYBRID_PREDICT_MAX_STEP_PX)
                {
                    reject_step = -HYBRID_PREDICT_MAX_STEP_PX;
                }
                rejected = 1;
                left = -1;
                right = -1;
                left_reacquired = 0;
                right_reacquired = 0;
            }
        }

        left_lost = (uint8_t)(left < 0);
        right_lost = (uint8_t)(right < 0);

        if (!left_lost || !right_lost)
        {
            if (diag != (hybrid_track_diag_t *)0)
            {
                /* 只有上方重新实测到边线，下方的预测行才算成功“桥接”。 */
                diag->bridged_rows = (uint8_t)(diag->bridged_rows + lost_run);
            }
            lost_run = 0;
            noseed_run = 0;
            last_observed_plus1 = (uint8_t)(r + 1u);
        }
        else
        {
            lost_run++;
            if (seed >= 0)
            {
                noseed_run = 0;
            }
            else
            {
                noseed_run++;
                if (noseed_run > HYBRID_MAX_GAP_ROWS)
                {
                    break;
                }
            }
            /* 锚定白列覆盖的行段本身就是实测自由空间；越过锚长后，
             * 连续双边丢失超过预算就停止——不给纯预测无限延伸的机会。 */
            if (lost_run > HYBRID_MAX_PREDICT_ROWS && r >= anchor_len)
            {
                break;
            }
            if (seed >= 0)
            {
                /* 白种子行：证明前方仍可行驶（自由空间），继续计入前瞻；
                 * 但它不是边线观测，不重置 lost_run。 */
                last_observed_plus1 = (uint8_t)(r + 1u);
            }
        }

        if (!left_lost && !right_lost)
        {
            last_width = (uint8_t)(right - left);
            mid = (int16_t)((left + right) / 2);
        }
        else if (!left_lost)
        {
            mid = (int16_t)(left + (int16_t)(last_width / 2u));
            right = (int16_t)(left + last_width);
        }
        else if (!right_lost)
        {
            mid = (int16_t)(right - (int16_t)(last_width / 2u));
            left = (int16_t)(right - last_width);
        }
        else
        {
            if (rejected)
            {
                mid = (int16_t)(previous_mid + reject_step);
            }
            else if (seed >= 0)
            {
                /* 跟随种子而不是旧斜率：种子是本行实测白像素，最多偏离走廊
                 * HYBRID_MAX_ROW_SHIFT，宽白区里表现为竖直走廊而非假想对角线。 */
                int16_t seed_step = (int16_t)(seed - previous_mid);
                if (seed_step > HYBRID_MAX_ROW_SHIFT)
                {
                    seed_step = HYBRID_MAX_ROW_SHIFT;
                }
                else if (seed_step < -HYBRID_MAX_ROW_SHIFT)
                {
                    seed_step = -HYBRID_MAX_ROW_SHIFT;
                }
                mid = (int16_t)(previous_mid + seed_step);
            }
            else
            {
                mid = predicted_mid;
            }
            left = (int16_t)(mid - (int16_t)(last_width / 2u));
            right = (int16_t)(left + last_width);
        }

        mid = clamp_col(mid);
        left = clamp_col(left);
        right = clamp_col(right);

        out->left[r] = (uint8_t)left;
        out->right[r] = (uint8_t)right;
        out->mid[r] = (uint8_t)mid;
        out->width[r] = last_width;
        out->left_lost[r] = left_lost;
        out->right_lost[r] = right_lost;

        if (diag != (hybrid_track_diag_t *)0)
        {
            if (left_reacquired)
            {
                diag->left_reacquires++;
            }
            if (right_reacquired)
            {
                diag->right_reacquires++;
            }
        }

        previous_mid2 = previous_mid;
        previous_mid = mid;
        previous_left2 = previous_left;
        previous_left = left;
        previous_right2 = previous_right;
        previous_right = right;
    }

    out->valid_rows = last_observed_plus1;
    out->both_lost_rows = 0;
    for (r = 0; r < out->valid_rows; r++)
    {
        if (out->left_lost[r] && out->right_lost[r])
        {
            out->both_lost_rows++;
        }
    }

    if (diag != (hybrid_track_diag_t *)0)
    {
        diag->traced_rows = out->valid_rows;
    }
}
