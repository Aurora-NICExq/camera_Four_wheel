#include <stdint.h>
#include "config.h"
#include "image.h"
#include "zf_common_headfile.h"

volatile int16_t  image_threshold  = 0;
volatile uint8_t  image_cross_fill = 1;
volatile uint16_t steer_far_w_pct  = STEER_FAR_W_PCT;

#define IMG_WHITE   (0xFFu)
#define IMG_BLACK   (0x00u)
#define TR_ROW(ir)  ((uint8_t)(IMG_H - 1u - (uint8_t)(ir)))

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

static int16_t g_err_hold;
static uint8_t g_hold_frames;
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
            longest_white_left_col = (int16_t)i;
        }
    }
    {
        int16_t right_len = 0;
        for (i = end_column; i >= start_column; i--)
        {
            if (right_len < white_column[i])
            {
                right_len = white_column[i];
                longest_white_right_col = (int16_t)i;
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
        left_line[i] = (int16_t)left_border;
        right_line[i] = (int16_t)right_border;
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
        left_line[i] = (int16_t)hx;
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
        right_line[i] = (int16_t)hx;
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
        left_line[i] = (int16_t)((i - start) * k + left_line[start]);
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
        right_line[i] = (int16_t)((i - start) * k + right_line[start]);
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
            left_down_find = (int16_t)i;
        }
        if (right_down_find == 0 &&
            (right_line[i] - right_line[i + 1] <= 5 && right_line[i + 1] - right_line[i + 2] <= 5 &&
             right_line[i + 2] - right_line[i + 3] <= 5) &&
            (right_line[i] - right_line[i - 2] <= -8) &&
            (right_line[i] - right_line[i - 3] <= -15) &&
            (right_line[i] - right_line[i - 4] <= -15))
        {
            right_down_find = (int16_t)i;
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
            left_up_find = (int16_t)i;
        }
        if (right_up_find == 0 &&
            (right_line[i] - right_line[i - 1] <= 5 && right_line[i - 1] - right_line[i - 2] <= 5 &&
             right_line[i - 2] - right_line[i - 3] <= 5) &&
            (right_line[i] - right_line[i + 2] <= -8) &&
            (right_line[i] - right_line[i + 3] <= -15) &&
            (right_line[i] - right_line[i + 4] <= -15))
        {
            right_up_find = (int16_t)i;
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
}

static void cross_detect(track_info_t *ti)
{
    int down_search_start;

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
}

static void image_filter(uint8_t bin[IMG_H][IMG_W])
{
    uint16_t r, c;
    for (r = 1; r < IMG_H - 1; r++)
    {
        for (c = 1; c < IMG_W - 1; c++)
        {
            uint32_t num =
                bin[r - 1][c - 1] + bin[r - 1][c] + bin[r - 1][c + 1]
              + bin[r][c - 1]     + bin[r][c + 1]
              + bin[r + 1][c - 1] + bin[r + 1][c] + bin[r + 1][c + 1];

            if (num >= (uint32_t)IMG_FILTER_SUM_MAX && bin[r][c] == IMG_BLACK)
            {
                bin[r][c] = IMG_WHITE;
            }
            if (num <= (uint32_t)IMG_FILTER_SUM_MIN && bin[r][c] == IMG_WHITE)
            {
                bin[r][c] = IMG_BLACK;
            }
        }
    }
}


static void export_track(track_info_t *ti)
{
    int     ir;
    uint8_t tr;
    uint8_t rows;
    uint8_t half_w;

    for (ir = 0; ir < IMG_H; ir++)
    {
        tr = TR_ROW(ir);
        ti->left[tr]  = clamp_u8(left_line[ir], 0, IMG_W - 1);
        ti->right[tr] = clamp_u8(right_line[ir], 0, IMG_W - 1);
        ti->left_lost[tr]  = (uint8_t)left_lost_flag[ir];
        ti->right_lost[tr] = (uint8_t)right_lost_flag[ir];
    }
    rows = (search_stop_line > 0) ? (uint8_t)search_stop_line : 0u;
    ti->valid_rows     = rows;
    ti->both_lost_rows = (uint8_t)both_lost_time;


    half_w = TRACK_HALF_W_FALLBACK;
    for (tr = 0; tr < rows; tr++)
    {
        if (!ti->left_lost[tr] && !ti->right_lost[tr] && ti->right[tr] > ti->left[tr])
        {
            half_w = (uint8_t)((uint16_t)(ti->right[tr] - ti->left[tr]) / 2u);
            break;
        }
    }


    for (tr = 0; tr < rows; tr++)
    {
        uint8_t ll = ti->left_lost[tr];
        uint8_t rl = ti->right_lost[tr];

        if (!ll && !rl)
        {
            ti->mid[tr] = (uint8_t)(((uint16_t)ti->left[tr] + (uint16_t)ti->right[tr]) / 2u);
            if (ti->right[tr] > ti->left[tr])
            {
                half_w = (uint8_t)((uint16_t)(ti->right[tr] - ti->left[tr]) / 2u);
            }
        }
        else if (!ll)
        {
            ti->mid[tr] = clamp_u8((int32_t)ti->left[tr] + (int32_t)half_w, 0, IMG_W - 1);
        }
        else if (!rl)
        {
            ti->mid[tr] = clamp_u8((int32_t)ti->right[tr] - (int32_t)half_w, 0, IMG_W - 1);
        }
        else
        {
            ti->mid[tr] = IMG_CENTER;
        }
    }
    for (tr = rows; tr < IMG_H; tr++)
    {
        ti->mid[tr] = IMG_CENTER;
    }
}


static int16_t two_band_error(track_info_t *ti)
{
    int32_t near_acc = 0;
    int32_t far_acc  = 0;
    uint8_t near_n   = 0;
    uint8_t far_n    = 0;
    uint8_t hi;
    uint8_t r;
    int32_t err;
    int32_t fw;


    hi = (ti->valid_rows < (uint8_t)STEER_FAR_ROW_HI)
       ? ti->valid_rows : (uint8_t)STEER_FAR_ROW_HI;

    for (r = 0; r < hi; r++)
    {
        int32_t dev;

        if (ti->left_lost[r] && ti->right_lost[r])
        {
            continue;
        }
        dev = (int32_t)ti->mid[r] - IMG_CENTER;
        if (r < (uint8_t)STEER_SPLIT_ROW)
        {
            near_acc += dev;
            near_n++;
        }
        else
        {
            far_acc += dev;
            far_n++;
        }
    }

    ti->near_rows = near_n;
    ti->far_rows  = far_n;

    if (near_n == 0u && far_n == 0u)
    {


        if (g_hold_frames < ERR_HOLD_MAX_FRAMES)
        {
            g_hold_frames++;
        }
        else
        {
            g_err_hold = (int16_t)((g_err_hold * 3) / 4);
        }
        return g_err_hold;
    }

    fw = (int32_t)steer_far_w_pct;
    if (fw > 100) fw = 100;

    if (far_n == 0u)
    {


        err = near_acc / (int32_t)near_n;
    }
    else if (near_n == 0u)
    {
        err = far_acc / (int32_t)far_n;
    }
    else
    {
        err = ((near_acc / (int32_t)near_n) * (100 - fw)
             + (far_acc  / (int32_t)far_n)  * fw) / 100;
    }

    g_hold_frames = 0;
    g_err_hold = (int16_t)err;
    return g_err_hold;
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

static void debug_draw_seg(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if (x0 == x1 && y0 == y1)
    {
        ips200_draw_point(x0, y0, color);
    }
    else
    {
        ips200_draw_line(x0, y0, x1, y1, color);
    }
}

static uint8_t g_calib_last_th = 0;

static uint8_t image_resolve_threshold(const uint8_t img[IMG_H][IMG_W])
{
    if (image_threshold > 0)
    {
        return (uint8_t)image_threshold;
    }
    return otsu_threshold(img);
}

uint8_t image_calib_last_th(void)
{
    return g_calib_last_th;
}


uint8_t image_calib_show(const uint8_t img[IMG_H][IMG_W])
{
    uint8_t th = image_resolve_threshold(img);

    g_calib_last_th = th;
    binarize(img, th);
    ips200_show_gray_image(0, 0, (const uint8 *)image_bin, IMG_W, IMG_H, IMG_W, IMG_H, 128);
    return th;
}

void image_debug_show(const track_info_t *ti)
{
    uint8_t tr;

    ips200_show_gray_image(0, 0, (const uint8 *)image_bin, IMG_W, IMG_H, IMG_W, IMG_H, 128);

    for (tr = 1; tr < ti->valid_rows; tr++)
    {
        uint16_t y0 = (uint16_t)(IMG_H - tr);
        uint16_t y1 = (uint16_t)(IMG_H - 1u - tr);
        uint8_t  filled0 = ti->cross_filled[tr - 1u];
        uint8_t  filled1 = ti->cross_filled[tr];

        if (!ti->left_lost[tr - 1u] && !ti->left_lost[tr])
        {
            debug_draw_seg(ti->left[tr - 1u], y0, ti->left[tr], y1,
                           (filled0 || filled1) ? RGB565_YELLOW : RGB565_BLUE);
        }
        if (!ti->right_lost[tr - 1u] && !ti->right_lost[tr])
        {
            debug_draw_seg(ti->right[tr - 1u], y0, ti->right[tr], y1,
                           (filled0 || filled1) ? RGB565_YELLOW : RGB565_RED);
        }
        if (!ti->left_lost[tr - 1u] && !ti->right_lost[tr - 1u] &&
            !ti->left_lost[tr] && !ti->right_lost[tr])
        {
            debug_draw_seg(ti->mid[tr - 1u], y0, ti->mid[tr], y1, RGB565_GREEN);
        }
    }

    if (ti->valid_rows == 1u)
    {
        uint16_t y = (uint16_t)(IMG_H - 1u);
        if (!ti->left_lost[0])
        {
            ips200_draw_point(ti->left[0], y,
                              ti->cross_filled[0] ? RGB565_YELLOW : RGB565_BLUE);
        }
        if (!ti->right_lost[0])
        {
            ips200_draw_point(ti->right[0], y,
                              ti->cross_filled[0] ? RGB565_YELLOW : RGB565_RED);
        }
        if (!ti->left_lost[0] && !ti->right_lost[0])
        {
            ips200_draw_point(ti->mid[0], y, RGB565_GREEN);
        }
    }
}


void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out)
{
    uint8_t th;

    init_cross_meta(out);

    th = (image_threshold > 0) ? (uint8_t)image_threshold : otsu_threshold(img);
    out->threshold = th;

    binarize(img, th);


    image_filter(image_bin);
    longest_white_column();
    if (image_cross_fill)
    {
        cross_detect(out);
    }
    export_track(out);

    out->error = two_band_error(out);


    out->err_hold = g_hold_frames;
}
