/* image.c - 18th 智能车：最长白列 + 十字拐点补线（移植自 the-18th-smartcar/Camera.c） */
#include <stdint.h>
#include "config.h"
#include "image.h"

volatile int16_t image_threshold = 0;

#define IMG_WHITE   (0xFFu)
#define IMG_BLACK   (0x00u)
#define TR_ROW(ir)  ((uint8_t)(IMG_H - 1u - (uint8_t)(ir))) /* 图像行 ir → 近车 track 行 */

static uint8_t image_bin[IMG_H][IMG_W];
static int16_t left_line[IMG_H];
static int16_t right_line[IMG_H];
static int16_t white_column[IMG_W];

static int16_t search_stop_line;
static int16_t longest_white_left_len;
static int16_t longest_white_left_col;
static int16_t longest_white_right_col;
static int16_t both_lost_time;
static int16_t left_lost_flag[IMG_H];
static int16_t right_lost_flag[IMG_H];

static int16_t cross_flag;
static int16_t left_up_find;
static int16_t left_down_find;
static int16_t right_up_find;
static int16_t right_down_find;

static uint8_t clamp_u8(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (uint8_t)lo;
    if (v > hi) return (uint8_t)hi;
    return (uint8_t)v;
}

static void init_cross_meta(track_info_t *ti)
{
    uint8_t r;
    ti->cross_valid = 0;
    ti->cross_lo = 0;
    ti->cross_hi = 0;
    ti->inflect_row = 0xFF;
    for (r = 0; r < IMG_H; r++)
    {
        ti->cross_filled[r] = 0;
    }
}

static uint8_t otsu_threshold(const uint8_t img[IMG_H][IMG_W])
{
    uint32_t hist[256] = {0};
    uint32_t total = 0;
    uint16_t r, c, i;

    for (r = 0; r < IMG_H; r += OTSU_ROW_STEP)
    {
        for (c = 0; c < IMG_W; c += OTSU_COL_STEP)
        {
            hist[img[r][c]]++;
            total++;
        }
    }

    uint64_t sum_all = 0;
    for (i = 0; i < 256; i++)
    {
        sum_all += (uint64_t)i * hist[i];
    }

    uint64_t best_var = 0;
    uint16_t best_th = FIXED_THRESHOLD;
    uint32_t w0 = 0;
    uint64_t sum0 = 0;

    for (i = 0; i < 256; i++)
    {
        w0 += hist[i];
        if (w0 == 0) continue;
        uint32_t w1 = total - w0;
        if (w1 == 0) break;
        sum0 += (uint64_t)i * hist[i];

        int64_t diff = (int64_t)(sum0 * w1) - (int64_t)((sum_all - sum0) * w0);
        uint64_t d2 = (uint64_t)((diff < 0) ? -diff : diff);
        uint64_t var = (d2 / w0) * (d2 / w1);
        if (var > best_var)
        {
            best_var = var;
            best_th = i;
        }
    }

    if (best_th < OTSU_THRESHOLD_MIN) best_th = OTSU_THRESHOLD_MIN;
    if (best_th > OTSU_THRESHOLD_MAX) best_th = OTSU_THRESHOLD_MAX;
    return (uint8_t)best_th;
}

static void binarize(const uint8_t img[IMG_H][IMG_W], uint8_t th)
{
    uint16_t r, c;
    for (r = 0; r < IMG_H; r++)
    {
        for (c = 0; c < IMG_W; c++)
        {
            image_bin[r][c] = (img[r][c] >= th) ? IMG_WHITE : IMG_BLACK;
        }
    }
}

static void longest_white_column(void)
{
    int i, j;
    int start_column = TH18_COL_MARGIN;
    int end_column = IMG_W - TH18_COL_MARGIN;
    int left_border = 0, right_border = 0;

    longest_white_left_len = 0;
    longest_white_left_col = 0;
    longest_white_right_col = 0;
    search_stop_line = 0;
    both_lost_time = 0;

    for (i = 0; i < IMG_H; i++)
    {
        left_lost_flag[i] = 0;
        right_lost_flag[i] = 0;
        left_line[i] = 0;
        right_line[i] = IMG_W - 1;
    }
    for (i = 0; i < IMG_W; i++)
    {
        white_column[i] = 0;
    }

    for (j = start_column; j <= end_column; j++)
    {
        for (i = IMG_H - 1; i >= 0; i--)
        {
            if (image_bin[i][j] == IMG_BLACK)
            {
                break;
            }
            white_column[j]++;
        }
    }

    for (i = start_column; i <= end_column; i++)
    {
        if (longest_white_left_len < white_column[i])
        {
            longest_white_left_len = white_column[i];
            longest_white_left_col = i;
        }
    }
    {
        int16_t right_len = 0;
        for (i = end_column; i >= start_column; i--)
        {
            if (right_len < white_column[i])
            {
                right_len = white_column[i];
                longest_white_right_col = i;
            }
        }
    }
    if (longest_white_right_col == 0)
    {
        longest_white_right_col = longest_white_left_col;
    }

    search_stop_line = longest_white_left_len;
    if (search_stop_line <= 0)
    {
        search_stop_line = 0;
        return;
    }
    if (search_stop_line > IMG_H)
    {
        search_stop_line = IMG_H;
    }

    for (i = IMG_H - 1; i >= IMG_H - search_stop_line; i--)
    {
        for (j = longest_white_right_col; j <= IMG_W - 1 - 2; j++)
        {
            if (image_bin[i][j] == IMG_WHITE &&
                image_bin[i][j + 1] == IMG_BLACK &&
                image_bin[i][j + 2] == IMG_BLACK)
            {
                right_border = j;
                right_lost_flag[i] = 0;
                break;
            }
            else if (j >= IMG_W - 1 - 2)
            {
                right_border = j;
                right_lost_flag[i] = 1;
                break;
            }
        }
        for (j = longest_white_left_col; j >= 2; j--)
        {
            if (image_bin[i][j] == IMG_WHITE &&
                image_bin[i][j - 1] == IMG_BLACK &&
                image_bin[i][j - 2] == IMG_BLACK)
            {
                left_border = j;
                left_lost_flag[i] = 0;
                break;
            }
            else if (j <= 2)
            {
                left_border = j;
                left_lost_flag[i] = 1;
                break;
            }
        }
        left_line[i] = left_border;
        right_line[i] = right_border;
    }

    for (i = IMG_H - 1; i >= 0; i--)
    {
        if (left_lost_flag[i] && right_lost_flag[i])
        {
            both_lost_time++;
        }
    }
}

static void left_add_line(int x1, int y1, int x2, int y2)
{
    int i, maxv, a1, a2, hx;

    if (x1 >= IMG_W - 1) x1 = IMG_W - 1;
    else if (x1 <= 0) x1 = 0;
    if (y1 >= IMG_H - 1) y1 = IMG_H - 1;
    else if (y1 <= 0) y1 = 0;
    if (x2 >= IMG_W - 1) x2 = IMG_W - 1;
    else if (x2 <= 0) x2 = 0;
    if (y2 >= IMG_H - 1) y2 = IMG_H - 1;
    else if (y2 <= 0) y2 = 0;

    a1 = y1;
    a2 = y2;
    if (a1 > a2)
    {
        maxv = a1;
        a1 = a2;
        a2 = maxv;
    }
    if (y2 == y1)
    {
        return;
    }
    for (i = a1; i <= a2; i++)
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= IMG_W) hx = IMG_W - 1;
        else if (hx <= 0) hx = 0;
        left_line[i] = hx;
    }
}

static void right_add_line(int x1, int y1, int x2, int y2)
{
    int i, maxv, a1, a2, hx;

    if (x1 >= IMG_W - 1) x1 = IMG_W - 1;
    else if (x1 <= 0) x1 = 0;
    if (y1 >= IMG_H - 1) y1 = IMG_H - 1;
    else if (y1 <= 0) y1 = 0;
    if (x2 >= IMG_W - 1) x2 = IMG_W - 1;
    else if (x2 <= 0) x2 = 0;
    if (y2 >= IMG_H - 1) y2 = IMG_H - 1;
    else if (y2 <= 0) y2 = 0;

    a1 = y1;
    a2 = y2;
    if (a1 > a2)
    {
        maxv = a1;
        a1 = a2;
        a2 = maxv;
    }
    if (y2 == y1)
    {
        return;
    }
    for (i = a1; i <= a2; i++)
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= IMG_W) hx = IMG_W - 1;
        else if (hx <= 0) hx = 0;
        right_line[i] = hx;
    }
}

static void lengthen_left_boundry(int start, int end)
{
    int i, t;
    int k;

    if (start >= IMG_H - 1) start = IMG_H - 1;
    else if (start <= 0) start = 0;
    if (end >= IMG_H - 1) end = IMG_H - 1;
    else if (end <= 0) end = 0;
    if (end < start)
    {
        t = end;
        end = start;
        start = t;
    }

    if (start <= 5)
    {
        left_add_line(left_line[start], start, left_line[end], end);
        return;
    }

    k = (left_line[start] - left_line[start - 4]) / 5;
    for (i = start; i <= end; i++)
    {
        left_line[i] = (i - start) * k + left_line[start];
        if (left_line[i] >= IMG_W - 1) left_line[i] = IMG_W - 1;
        else if (left_line[i] <= 0) left_line[i] = 0;
    }
}

static void lengthen_right_boundry(int start, int end)
{
    int i, t;
    int k;

    if (start >= IMG_H - 1) start = IMG_H - 1;
    else if (start <= 0) start = 0;
    if (end >= IMG_H - 1) end = IMG_H - 1;
    else if (end <= 0) end = 0;
    if (end < start)
    {
        t = end;
        end = start;
        start = t;
    }

    if (start <= 5)
    {
        right_add_line(right_line[start], start, right_line[end], end);
        return;
    }

    k = (right_line[start] - right_line[start - 4]) / 5;
    for (i = start; i <= end; i++)
    {
        right_line[i] = (i - start) * k + right_line[start];
        if (right_line[i] >= IMG_W - 1) right_line[i] = IMG_W - 1;
        else if (right_line[i] <= 0) right_line[i] = 0;
    }
}

static void find_down_point(int start, int end)
{
    int i, t;

    left_down_find = 0;
    right_down_find = 0;
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (start >= IMG_H - 1 - 5) start = IMG_H - 1 - 5;
    if (end <= IMG_H - search_stop_line) end = IMG_H - search_stop_line;
    if (end <= 5) end = 5;

    for (i = start; i >= end; i--)
    {
        if (left_down_find == 0 &&
            (left_line[i] - left_line[i + 1] <= 5 && left_line[i + 1] - left_line[i + 2] <= 5 &&
             left_line[i + 2] - left_line[i + 3] <= 5) &&
            (left_line[i] - left_line[i - 2] >= 8) &&
            (left_line[i] - left_line[i - 3] >= 15) &&
            (left_line[i] - left_line[i - 4] >= 15))
        {
            left_down_find = i;
        }
        if (right_down_find == 0 &&
            (right_line[i] - right_line[i + 1] <= 5 && right_line[i + 1] - right_line[i + 2] <= 5 &&
             right_line[i + 2] - right_line[i + 3] <= 5) &&
            (right_line[i] - right_line[i - 2] <= -8) &&
            (right_line[i] - right_line[i - 3] <= -15) &&
            (right_line[i] - right_line[i - 4] <= -15))
        {
            right_down_find = i;
        }
        if (left_down_find != 0 && right_down_find != 0)
        {
            break;
        }
    }
}

static void find_up_point(int start, int end)
{
    int i, t;

    left_up_find = 0;
    right_up_find = 0;
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (end <= IMG_H - search_stop_line) end = IMG_H - search_stop_line;
    if (end <= 5) end = 5;
    if (start >= IMG_H - 1 - 5) start = IMG_H - 1 - 5;

    for (i = start; i >= end; i--)
    {
        if (left_up_find == 0 &&
            (left_line[i] - left_line[i - 1] <= 5 && left_line[i - 1] - left_line[i - 2] <= 5 &&
             left_line[i - 2] - left_line[i - 3] <= 5) &&
            (left_line[i] - left_line[i + 2] >= 8) &&
            (left_line[i] - left_line[i + 3] >= 15) &&
            (left_line[i] - left_line[i + 4] >= 15))
        {
            left_up_find = i;
        }
        if (right_up_find == 0 &&
            (right_line[i] - right_line[i - 1] <= 5 && right_line[i - 1] - right_line[i - 2] <= 5 &&
             right_line[i - 2] - right_line[i - 3] <= 5) &&
            (right_line[i] - right_line[i + 2] <= -8) &&
            (right_line[i] - right_line[i + 3] <= -15) &&
            (right_line[i] - right_line[i + 4] <= -15))
        {
            right_up_find = i;
        }
        if (left_up_find != 0 && right_up_find != 0)
        {
            break;
        }
    }
    if ((left_up_find > 0 && right_up_find > 0) &&
        (left_up_find - right_up_find >= 30 || right_up_find - left_up_find >= 30))
    {
        left_up_find = 0;
        right_up_find = 0;
    }
}

static void mark_cross_fill(int y1, int y2, track_info_t *ti)
{
    int a1 = y1;
    int a2 = y2;
    int i;
    if (a1 > a2)
    {
        int t = a1;
        a1 = a2;
        a2 = t;
    }
    for (i = a1; i <= a2; i++)
    {
        uint8_t tr = TR_ROW(i);
        if (tr < IMG_H)
        {
            ti->cross_filled[tr] = 1;
        }
    }
    if (ti->cross_lo == 0 && ti->cross_hi == 0)
    {
        ti->cross_lo = TR_ROW(a2);
        ti->cross_hi = (uint8_t)(TR_ROW(a1) + 1u);
    }
}

static void cross_detect(track_info_t *ti)
{
    int down_search_start;

    cross_flag = 0;
    left_up_find = 0;
    right_up_find = 0;
    left_down_find = 0;
    right_down_find = 0;

    if (both_lost_time < TH18_CROSS_BOTH_LOST_MIN)
    {
        return;
    }

    find_up_point(IMG_H - 1, 0);
    if (left_up_find == 0 && right_up_find == 0)
    {
        return;
    }
    if (left_up_find == 0 || right_up_find == 0)
    {
        return;
    }

    cross_flag = 1;
    down_search_start = (left_up_find > right_up_find) ? left_up_find : right_up_find;
    find_down_point(IMG_H - 5, down_search_start + 2);

    if (left_down_find <= left_up_find) left_down_find = 0;
    if (right_down_find <= right_up_find) right_down_find = 0;

    if (left_down_find != 0 && right_down_find != 0)
    {
        left_add_line(left_line[left_up_find], left_up_find,
                      left_line[left_down_find], left_down_find);
        right_add_line(right_line[right_up_find], right_up_find,
                       right_line[right_down_find], right_down_find);
        mark_cross_fill(left_up_find, left_down_find, ti);
        mark_cross_fill(right_up_find, right_down_find, ti);
    }
    else if (left_down_find == 0 && right_down_find != 0)
    {
        lengthen_left_boundry(left_up_find - 1, IMG_H - 1);
        right_add_line(right_line[right_up_find], right_up_find,
                       right_line[right_down_find], right_down_find);
        mark_cross_fill(left_up_find, IMG_H - 1, ti);
        mark_cross_fill(right_up_find, right_down_find, ti);
    }
    else if (left_down_find != 0 && right_down_find == 0)
    {
        left_add_line(left_line[left_up_find], left_up_find,
                      left_line[left_down_find], left_down_find);
        lengthen_right_boundry(right_up_find - 1, IMG_H - 1);
        mark_cross_fill(left_up_find, left_down_find, ti);
        mark_cross_fill(right_up_find, IMG_H - 1, ti);
    }
    else
    {
        lengthen_left_boundry(left_up_find - 1, IMG_H - 1);
        lengthen_right_boundry(right_up_find - 1, IMG_H - 1);
        mark_cross_fill(left_up_find, IMG_H - 1, ti);
        mark_cross_fill(right_up_find, IMG_H - 1, ti);
    }

    ti->cross_valid = 1;
    ti->inflect_row = TR_ROW(left_up_find);
}

static void export_track(track_info_t *ti)
{
    int ir;
    for (ir = 0; ir < IMG_H; ir++)
    {
        uint8_t tr = TR_ROW(ir);
        ti->left[tr] = clamp_u8(left_line[ir], 0, IMG_W - 1);
        ti->right[tr] = clamp_u8(right_line[ir], 0, IMG_W - 1);
        ti->mid[tr] = (uint8_t)(((uint16_t)ti->left[tr] + (uint16_t)ti->right[tr]) / 2u);
        ti->width[tr] = (uint8_t)(ti->right[tr] - ti->left[tr]);
        ti->left_lost[tr] = (uint8_t)left_lost_flag[ir];
        ti->right_lost[tr] = (uint8_t)right_lost_flag[ir];
    }

    ti->valid_rows = (search_stop_line > 0) ? (uint8_t)search_stop_line : 0u;
    ti->longest_col = (uint8_t)longest_white_left_col;
    ti->both_lost_rows = (uint8_t)both_lost_time;
}

static int16_t weighted_error(const track_info_t *ti, uint16_t duty_now)
{
    static const uint8_t w_low[STEER_W_BANDS]  = STEER_WEIGHTS_LOWSPEED;
    static const uint8_t w_high[STEER_W_BANDS] = STEER_WEIGHTS_HIGHSPEED;

    uint32_t k = ((uint32_t)duty_now * 256u) / DUTY_HARD_CAP;
    if (k > 256u) k = 256u;

    int32_t acc = 0, w_sum = 0;
    uint8_t r;

    for (r = 0; r < ti->valid_rows; r++)
    {
        uint8_t band = (uint8_t)(r / STEER_W_BAND_ROWS);
        if (band >= STEER_W_BANDS) band = STEER_W_BANDS - 1;
        int32_t w = (int32_t)w_low[band] * (int32_t)(256u - k)
                  + (int32_t)w_high[band] * (int32_t)k;

        if (ti->cross_filled[r])
        {
            w = (w * STEER_W_CROSS_FILL_PCT) / 100;
        }
        else if (ti->left_lost[r] && ti->right_lost[r])
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
    if (w_sum == 0) return 0;
    return (int16_t)(acc / w_sum);
}

static int16_t segment_slope_q8(const track_info_t *ti, uint8_t lo, uint8_t hi, uint8_t *ok)
{
    *ok = 0;
    if (hi >= ti->valid_rows) hi = (uint8_t)(ti->valid_rows - 1u);
    while (lo < hi && ti->left_lost[lo] && ti->right_lost[lo]) lo++;
    while (hi > lo && ti->left_lost[hi] && ti->right_lost[hi]) hi--;
    if (hi <= lo || (uint8_t)(hi - lo) < CURV_MIN_SPAN_ROWS ||
        (ti->left_lost[lo] && ti->right_lost[lo]))
    {
        return 0;
    }
    *ok = 1;
    return (int16_t)((((int32_t)ti->mid[hi] - (int32_t)ti->mid[lo]) * 256) /
                     (int32_t)(hi - lo));
}

uint8_t detect_cross(const track_info_t *ti)
{
#if ENABLE_CROSS
    return ti->cross_valid;
#else
    (void)ti;
    return 0;
#endif
}

static uint8_t ring_side_signature(const track_info_t *ti,
                                   const uint8_t *arc_lost, const uint8_t *solid_lost,
                                   uint8_t *gap_lo)
{
    uint8_t r;
    uint8_t hi = (ti->valid_rows < RING_BAND_ROW_HI) ? ti->valid_rows : RING_BAND_ROW_HI;
    uint8_t solid_lost_cnt = 0;

    for (r = RING_BAND_ROW_LO; r < hi; r++)
    {
        if (solid_lost[r]) solid_lost_cnt++;
    }
    if (solid_lost_cnt > RING_SOLID_MAX_LOST) return 0;

    for (r = RING_BAND_ROW_LO; r < hi; r++)
    {
        uint8_t end, good_below = 0, good_above = 0;
        int16_t rr;

        if (!arc_lost[r]) continue;
        end = r;
        while (end < hi && arc_lost[end]) end++;

        for (rr = (int16_t)r - 1; rr >= RING_BAND_ROW_LO; rr--)
        {
            if (arc_lost[rr]) break;
            good_below++;
        }
        for (rr = end; rr < hi; rr++)
        {
            if (arc_lost[rr]) break;
            good_above++;
        }

        if ((uint8_t)(end - r) >= RING_ARC_MIN_LOST &&
            good_below >= RING_ARC_MIN_GOOD_BELOW &&
            good_above >= RING_ARC_MIN_GOOD_ABOVE)
        {
            *gap_lo = r;
            return 1;
        }
        if (end > r) r = (uint8_t)(end - 1u);
    }
    return 0;
}

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

uint8_t detect_ramp(const track_info_t *ti)
{
#if ENABLE_RAMP
    uint8_t r, wide_rows = 0;
    uint8_t hi = (ti->valid_rows < RAMP_BAND_ROW_HI) ? ti->valid_rows : RAMP_BAND_ROW_HI;
    for (r = RAMP_BAND_ROW_LO; r < hi; r++)
    {
        if (!ti->left_lost[r] && !ti->right_lost[r])
        {
            uint16_t measured = (uint16_t)(ti->right[r] - ti->left[r]);
            if (measured * 100u > (uint16_t)ti->width[r] * RAMP_WIDTH_NUM) wide_rows++;
        }
    }
    return (uint8_t)(wide_rows >= RAMP_MIN_ROWS);
#else
    (void)ti;
    return 0;
#endif
}

uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe)
{
    *severe = 0;
    if (ti->valid_rows < FAILSAFE_MIN_ROWS)
    {
        *severe = (uint8_t)(ti->valid_rows == 0u);
        return 1;
    }
    {
        uint16_t lost_pct_lhs = (uint16_t)ti->both_lost_rows * 100u;
        uint16_t rows_rhs = (uint16_t)ti->valid_rows;
        if (lost_pct_lhs >= rows_rhs * FAILSAFE_SEVERE_BOTH_LOST_PCT)
        {
            *severe = 1;
            return 1;
        }
        return (uint8_t)(lost_pct_lhs >= rows_rhs * FAILSAFE_MAX_BOTH_LOST_PCT);
    }
}

void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out)
{
    uint8_t th;

    init_cross_meta(out);

    th = (image_threshold > 0) ? (uint8_t)image_threshold : otsu_threshold(img);
    out->threshold = th;

    binarize(img, th);
    longest_white_column();
    cross_detect(out);
    export_track(out);

    out->det_cross = (uint8_t)cross_flag;
    out->det_ring_left = detect_ring_left(out);
    out->det_ring_right = detect_ring_right(out);
    out->det_ramp = detect_ramp(out);
    out->det_value = (int16_t)out->both_lost_rows;

    if (out->valid_rows > CURV_FAR_ROW_LO)
    {
        uint8_t near_ok, far_ok;
        int16_t s_near = segment_slope_q8(out, CURV_NEAR_ROW_LO, CURV_NEAR_ROW_HI, &near_ok);
        int16_t s_far  = segment_slope_q8(out, CURV_FAR_ROW_LO,  CURV_FAR_ROW_HI,  &far_ok);
        out->curvature = (near_ok && far_ok) ? (int16_t)(s_far - s_near) : 0;
    }
    else
    {
        out->curvature = 0;
    }

    out->error = weighted_error(out, duty_now);
}
