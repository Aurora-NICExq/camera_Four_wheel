/* image.c - 八邻域 v2.0 双边跟踪 + 最小二乘十字补线（Develop/八邻域_v2.0） */
#include <stdint.h>
#include "config.h"
#include "image.h"

volatile int16_t image_threshold = 0;

#define IMG_WHITE   (0xFFu)
#define IMG_BLACK   (0x00u)
#define TR_ROW(ir)  ((uint8_t)(IMG_H - 1u - (uint8_t)(ir))) /* 图像行 ir → 近车 track 行 */

static uint8_t  image_bin[IMG_H][IMG_W];
static uint8_t  l_border[IMG_H];
static uint8_t  r_border[IMG_H];

static uint16_t points_l[EIGHTN_MAX_POINTS][2];
static uint16_t points_r[EIGHTN_MAX_POINTS][2];
static uint16_t dir_l[EIGHTN_MAX_POINTS];
static uint16_t dir_r[EIGHTN_MAX_POINTS];

static uint8_t  start_point_l[2];
static uint8_t  start_point_r[2];
static uint16_t data_stastics_l;
static uint16_t data_stastics_r;
static uint8_t  hightest_row;
static uint8_t  cross_break_l;
static uint8_t  cross_break_r;
static uint8_t  cross_flag;

static int my_abs(int v)
{
    return (v >= 0) ? v : -v;
}

static int16_t limit_a_b(int16_t x, int16_t a, int16_t b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

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

            if (num >= (uint32_t)EIGHTN_FILTER_SUM_MAX && bin[r][c] == IMG_BLACK)
            {
                bin[r][c] = IMG_WHITE;
            }
            if (num <= (uint32_t)EIGHTN_FILTER_SUM_MIN && bin[r][c] == IMG_WHITE)
            {
                bin[r][c] = IMG_BLACK;
            }
        }
    }
}

static void image_draw_rectan(uint8_t bin[IMG_H][IMG_W])
{
    uint16_t r, c;
    for (r = 0; r < IMG_H; r++)
    {
        bin[r][0] = IMG_BLACK;
        bin[r][1] = IMG_BLACK;
        bin[r][IMG_W - 1] = IMG_BLACK;
        bin[r][IMG_W - 2] = IMG_BLACK;
    }
    for (c = 0; c < IMG_W; c++)
    {
        bin[0][c] = IMG_BLACK;
        bin[1][c] = IMG_BLACK;
    }
}

static uint8_t get_start_point(uint8_t start_row)
{
    uint16_t i;
    uint8_t l_found = 0;
    uint8_t r_found = 0;

    start_point_l[0] = 0;
    start_point_l[1] = 0;
    start_point_r[0] = 0;
    start_point_r[1] = 0;

    for (i = IMG_W / 2; i > EIGHTN_BORDER_MIN; i--)
    {
        start_point_l[0] = (uint8_t)i;
        start_point_l[1] = start_row;
        if (image_bin[start_row][i] == IMG_WHITE &&
            image_bin[start_row][i - 1] == IMG_BLACK)
        {
            l_found = 1;
            break;
        }
    }

    for (i = IMG_W / 2; i < EIGHTN_BORDER_MAX; i++)
    {
        start_point_r[0] = (uint8_t)i;
        start_point_r[1] = start_row;
        if (image_bin[start_row][i] == IMG_WHITE &&
            image_bin[start_row][i + 1] == IMG_BLACK)
        {
            r_found = 1;
            break;
        }
    }

    return (uint8_t)(l_found && r_found);
}

static void search_l_r(uint16_t break_flag,
                       uint8_t bin[IMG_H][IMG_W],
                       uint16_t *l_stastic,
                       uint16_t *r_stastic,
                       uint8_t l_start_x,
                       uint8_t l_start_y,
                       uint8_t r_start_x,
                       uint8_t r_start_y,
                       uint8_t *hightest)
{
    static const int8_t seeds_l[8][2] = {
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}
    };
    static const int8_t seeds_r[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };

    uint8_t  search_filds_l[8][2];
    uint8_t  search_filds_r[8][2];
    uint8_t  temp_l[8][2];
    uint8_t  temp_r[8][2];
    uint8_t  center_point_l[2];
    uint8_t  center_point_r[2];
    uint8_t  index_l;
    uint8_t  index_r;
    uint8_t  i;
    uint8_t  j;
    uint16_t l_data_statics = *l_stastic;
    uint16_t r_data_statics = *r_stastic;

    center_point_l[0] = l_start_x;
    center_point_l[1] = l_start_y;
    center_point_r[0] = r_start_x;
    center_point_r[1] = r_start_y;

    while (break_flag--)
    {
        for (i = 0; i < 8; i++)
        {
            search_filds_l[i][0] = (uint8_t)(center_point_l[0] + seeds_l[i][0]);
            search_filds_l[i][1] = (uint8_t)(center_point_l[1] + seeds_l[i][1]);
        }
        points_l[l_data_statics][0] = center_point_l[0];
        points_l[l_data_statics][1] = center_point_l[1];
        l_data_statics++;

        for (i = 0; i < 8; i++)
        {
            search_filds_r[i][0] = (uint8_t)(center_point_r[0] + seeds_r[i][0]);
            search_filds_r[i][1] = (uint8_t)(center_point_r[1] + seeds_r[i][1]);
        }
        points_r[r_data_statics][0] = center_point_r[0];
        points_r[r_data_statics][1] = center_point_r[1];

        index_l = 0;
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;
            temp_l[i][1] = 0;
        }

        for (i = 0; i < 8; i++)
        {
            if (bin[search_filds_l[i][1]][search_filds_l[i][0]] == IMG_BLACK &&
                bin[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == IMG_WHITE)
            {
                temp_l[index_l][0] = search_filds_l[i][0];
                temp_l[index_l][1] = search_filds_l[i][1];
                dir_l[l_data_statics - 1] = i;
                index_l++;
            }

            if (index_l)
            {
                center_point_l[0] = temp_l[0][0];
                center_point_l[1] = temp_l[0][1];
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];
                        center_point_l[1] = temp_l[j][1];
                    }
                }
            }
        }

        if ((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] &&
             points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] &&
             points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] &&
             points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) ||
            (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
             points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
             points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
             points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1]))
        {
            break;
        }

        if (my_abs((int)points_r[r_data_statics][0] - (int)points_l[l_data_statics - 1][0]) < EIGHTN_MEET_DIST &&
            my_abs((int)points_r[r_data_statics][1] - (int)points_l[l_data_statics - 1][1]) < EIGHTN_MEET_DIST)
        {
            *hightest = (uint8_t)((points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1);
            break;
        }

        if (points_r[r_data_statics][1] < points_l[l_data_statics - 1][1])
        {
            continue;
        }

        if (dir_l[l_data_statics - 1] == 7 &&
            points_r[r_data_statics][1] > points_l[l_data_statics - 1][1])
        {
            center_point_l[0] = (uint8_t)points_l[l_data_statics - 1][0];
            center_point_l[1] = (uint8_t)points_l[l_data_statics - 1][1];
            l_data_statics--;
        }
        r_data_statics++;

        index_r = 0;
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;
            temp_r[i][1] = 0;
        }

        for (i = 0; i < 8; i++)
        {
            if (bin[search_filds_r[i][1]][search_filds_r[i][0]] == IMG_BLACK &&
                bin[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == IMG_WHITE)
            {
                temp_r[index_r][0] = search_filds_r[i][0];
                temp_r[index_r][1] = search_filds_r[i][1];
                dir_r[r_data_statics - 1] = i;
                index_r++;
            }

            if (index_r)
            {
                center_point_r[0] = temp_r[0][0];
                center_point_r[1] = temp_r[0][1];
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];
                        center_point_r[1] = temp_r[j][1];
                    }
                }
            }
        }
    }

    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

static void get_left(uint16_t total_l)
{
    uint16_t j;
    int16_t h = (int16_t)EIGHTN_START_ROW;
    uint8_t i;

    for (i = 0; i < IMG_H; i++)
    {
        l_border[i] = EIGHTN_BORDER_MIN;
    }

    for (j = 0; j < total_l; j++)
    {
        if (points_l[j][1] == (uint16_t)h)
        {
            l_border[h] = (uint8_t)(points_l[j][0] + 1u);
        }
        else
        {
            continue;
        }
        h--;
        if (h <= 0)
        {
            break;
        }
    }
}

static void get_right(uint16_t total_r)
{
    uint16_t j;
    int16_t h = (int16_t)EIGHTN_START_ROW;
    uint8_t i;

    for (i = 0; i < IMG_H; i++)
    {
        r_border[i] = EIGHTN_BORDER_MAX;
    }

    for (j = 0; j < total_r; j++)
    {
        if (points_r[j][1] == (uint16_t)h)
        {
            r_border[h] = (uint8_t)(points_r[j][0] - 1u);
        }
        else
        {
            continue;
        }
        h--;
        if (h <= 0)
        {
            break;
        }
    }
}

static float slope_calculate(uint8_t begin, uint8_t end, const uint8_t *border)
{
    float xsum = 0.0f;
    float ysum = 0.0f;
    float xysum = 0.0f;
    float x2sum = 0.0f;
    int16_t i;
    float result = 0.0f;
    static float result_last;

    for (i = (int16_t)begin; i < (int16_t)end; i++)
    {
        xsum  += (float)i;
        ysum  += (float)border[i];
        xysum += (float)i * (float)border[i];
        x2sum += (float)i * (float)i;
    }

    {
        float denom = ((float)(end - begin) * x2sum - xsum * xsum);
        if (denom != 0.0f)
        {
            result = (((float)(end - begin) * xysum - xsum * ysum) / denom);
            result_last = result;
        }
        else
        {
            result = result_last;
        }
    }
    return result;
}

static void calculate_s_i(uint8_t start, uint8_t end, const uint8_t *border,
                          float *slope_rate, float *intercept)
{
    uint16_t i;
    uint16_t num = 0;
    float x_average;
    float y_average;
    uint32_t xsum = 0;
    uint32_t ysum = 0;

    for (i = start; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }

    if (num)
    {
        x_average = (float)xsum / (float)num;
        y_average = (float)ysum / (float)num;
    }
    else
    {
        x_average = 0.0f;
        y_average = 0.0f;
    }

    *slope_rate = slope_calculate(start, end, border);
    *intercept = y_average - (*slope_rate) * x_average;
}

static void mark_cross_fill_rows(uint8_t y_lo, uint8_t y_hi, track_info_t *ti)
{
    uint8_t a1 = y_lo;
    uint8_t a2 = y_hi;
    uint8_t i;

    if (a1 > a2)
    {
        uint8_t t = a1;
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

static void cross_fill(uint8_t bin[IMG_H][IMG_W], track_info_t *ti)
{
    uint16_t i;
    uint8_t start;
    uint8_t end;
    float slope_rate = 0.0f;
    float intercept = 0.0f;
    uint8_t fill_from;

    cross_break_l = 0;
    cross_break_r = 0;
    cross_flag = 0;

    for (i = 1; i + 7u < data_stastics_l; i++)
    {
        if (dir_l[i - 1] == 4 && dir_l[i] == 4 &&
            dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
        {
            cross_break_l = (uint8_t)points_l[i][1];
            break;
        }
    }

    for (i = 1; i + 7u < data_stastics_r; i++)
    {
        if (dir_r[i - 1] == 4 && dir_r[i] == 4 &&
            dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
        {
            cross_break_r = (uint8_t)points_r[i][1];
            break;
        }
    }

    if (!cross_break_l || !cross_break_r)
    {
        return;
    }

    if (!bin[IMG_H - 1][EIGHTN_CROSS_CORNER_L] ||
        !bin[IMG_H - 1][EIGHTN_CROSS_CORNER_R])
    {
        return;
    }

    cross_flag = 1;
    fill_from = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_NEAR);

    start = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_BACK);
    start = (uint8_t)limit_a_b((int16_t)start, 0, IMG_H - 1);
    end = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_NEAR);
    calculate_s_i(start, end, l_border, &slope_rate, &intercept);
    for (i = fill_from; i < (uint16_t)(IMG_H - 1); i++)
    {
        int16_t v = (int16_t)(slope_rate * (float)i + intercept);
        l_border[i] = (uint8_t)limit_a_b(v, EIGHTN_BORDER_MIN, EIGHTN_BORDER_MAX);
        mark_cross_fill_rows((uint8_t)i, (uint8_t)i, ti);
    }

    start = (uint8_t)(cross_break_r - EIGHTN_CROSS_SLOPE_BACK);
    start = (uint8_t)limit_a_b((int16_t)start, 0, IMG_H - 1);
    end = (uint8_t)(cross_break_r - EIGHTN_CROSS_SLOPE_NEAR);
    calculate_s_i(start, end, r_border, &slope_rate, &intercept);
    fill_from = (uint8_t)(cross_break_r - EIGHTN_CROSS_SLOPE_NEAR);
    for (i = fill_from; i < (uint16_t)(IMG_H - 1); i++)
    {
        int16_t v = (int16_t)(slope_rate * (float)i + intercept);
        r_border[i] = (uint8_t)limit_a_b(v, EIGHTN_BORDER_MIN, EIGHTN_BORDER_MAX);
        mark_cross_fill_rows((uint8_t)i, (uint8_t)i, ti);
    }

    ti->cross_valid = 1;
    ti->inflect_row = TR_ROW(cross_break_l);
}

static void export_track(track_info_t *ti, uint8_t hightest)
{
    uint8_t ir;
    uint8_t tr;
    uint8_t both_lost = 0;
    uint8_t lo = hightest;
    uint8_t hi = EIGHTN_START_ROW;

    for (tr = 0; tr < IMG_H; tr++)
    {
        ti->left[tr] = 0;
        ti->right[tr] = (uint8_t)(IMG_W - 1);
        ti->mid[tr] = IMG_CENTER;
        ti->width[tr] = 0;
        ti->left_lost[tr] = 1;
        ti->right_lost[tr] = 1;
    }

    if (lo > hi)
    {
        ti->valid_rows = 0;
        ti->longest_col = IMG_CENTER;
        ti->both_lost_rows = 0;
        return;
    }

    for (ir = lo; ir <= hi; ir++)
    {
        tr = TR_ROW(ir);
        ti->left[tr] = clamp_u8(l_border[ir], 0, IMG_W - 1);
        ti->right[tr] = clamp_u8(r_border[ir], 0, IMG_W - 1);
        ti->mid[tr] = (uint8_t)(((uint16_t)ti->left[tr] + (uint16_t)ti->right[tr]) / 2u);
        ti->width[tr] = (uint8_t)(ti->right[tr] - ti->left[tr]);
        ti->left_lost[tr] = (uint8_t)(l_border[ir] <= EIGHTN_BORDER_MIN);
        ti->right_lost[tr] = (uint8_t)(r_border[ir] >= EIGHTN_BORDER_MAX);
        if (ti->left_lost[tr] && ti->right_lost[tr])
        {
            both_lost++;
        }
    }

    ti->valid_rows = (uint8_t)(hi - lo + 1u);
    ti->longest_col = start_point_l[0];
    ti->both_lost_rows = both_lost;
}

static int16_t weighted_error(const track_info_t *ti, uint16_t duty_now)
{
    static const uint8_t w_low[STEER_W_BANDS]  = STEER_WEIGHTS_LOWSPEED;
    static const uint8_t w_high[STEER_W_BANDS] = STEER_WEIGHTS_HIGHSPEED;

    uint32_t k = ((uint32_t)duty_now * 256u) / DUTY_HARD_CAP;
    if (k > 256u) k = 256u;

    int32_t acc = 0;
    int32_t w_sum = 0;
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
        uint8_t end;
        uint8_t good_below = 0;
        uint8_t good_above = 0;
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
    uint8_t r;
    uint8_t wide_rows = 0;
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
    cross_flag = 0;
    hightest_row = 0;
    data_stastics_l = 0;
    data_stastics_r = 0;

    th = (image_threshold > 0) ? (uint8_t)image_threshold : otsu_threshold(img);
    out->threshold = th;

    binarize(img, th);
    image_filter(image_bin);
    image_draw_rectan(image_bin);

    if (get_start_point(EIGHTN_START_ROW))
    {
        search_l_r((uint16_t)EIGHTN_MAX_POINTS, image_bin,
                   &data_stastics_l, &data_stastics_r,
                   start_point_l[0], start_point_l[1],
                   start_point_r[0], start_point_r[1],
                   &hightest_row);
        get_left(data_stastics_l);
        get_right(data_stastics_r);
        cross_fill(image_bin, out);
        export_track(out, hightest_row);
    }
    else
    {
        export_track(out, EIGHTN_START_ROW + 1u); /* lo > hi → valid_rows = 0 */
    }

    out->det_cross = cross_flag;
    out->det_ring_left = detect_ring_left(out);
    out->det_ring_right = detect_ring_right(out);
    out->det_ramp = detect_ramp(out);
    out->det_value = (int16_t)out->both_lost_rows;

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

    out->error = weighted_error(out, duty_now);
}
