/* image.c - 八邻域双边巡线 */
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

static uint8_t  start_point_l[2];
static uint8_t  start_point_r[2];
static uint16_t data_stastics_l;
static uint16_t data_stastics_r;
static uint8_t  hightest_row;

static int my_abs(int v)
{
    return (v >= 0) ? v : -v;
}

static uint8_t clamp_u8(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (uint8_t)lo;
    if (v > hi) return (uint8_t)hi;
    return (uint8_t)v;
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

        if ((r_data_statics >= 2u &&
             points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] &&
             points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] &&
             points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] &&
             points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) ||
            (l_data_statics >= 3u &&
             points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
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
        ti->left_lost[tr] = (uint8_t)(l_border[ir] <= (EIGHTN_BORDER_MIN + EIGHTN_EDGE_LOST_MARGIN));
        ti->right_lost[tr] = (uint8_t)(r_border[ir] >= (EIGHTN_BORDER_MAX - EIGHTN_EDGE_LOST_MARGIN));
        if (ti->left_lost[tr] && ti->right_lost[tr])
        {
            both_lost++;
        }
    }

    ti->valid_rows = (uint8_t)(hi - lo + 1u);

    /* 裁掉远端连续双丢行:搜索未到达或沿图像黑框爬行的行只有假居中数据,
       留在 valid_rows 里会稀释 weighted_error,并让 curve_temp 在入弯口失明 */
    while (ti->valid_rows > 0u)
    {
        uint8_t far_tr = (uint8_t)(TR_ROW(EIGHTN_START_ROW) + ti->valid_rows - 1u);
        if (!(ti->left_lost[far_tr] && ti->right_lost[far_tr]))
        {
            break;
        }
        ti->valid_rows--;
        both_lost--;
    }
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

    /* export_track 从 TR_ROW(EIGHTN_START_ROW) 起写入,track 行 0 无数据;
       band 仍按相对行号划分 */
    for (r = 0; r < ti->valid_rows; r++)
    {
        uint8_t tr = (uint8_t)(TR_ROW(EIGHTN_START_ROW) + r);
        uint8_t band = (uint8_t)(r / STEER_W_BAND_ROWS);
        if (band >= STEER_W_BANDS) band = STEER_W_BANDS - 1;
        int32_t w = (int32_t)w_low[band] * (int32_t)(256u - k)
                  + (int32_t)w_high[band] * (int32_t)k;

        if (ti->left_lost[tr] && ti->right_lost[tr])
        {
            w = (w * STEER_W_BOTH_LOST_PCT) / 100;
        }
        else if (ti->left_lost[tr] || ti->right_lost[tr])
        {
            w = (w * STEER_W_SINGLE_EDGE_PCT) / 100;
        }
        acc   += w * ((int16_t)ti->mid[tr] - IMG_CENTER);
        w_sum += w;
    }
    if (w_sum == 0) return 0;
    return (int16_t)(acc / w_sum);
}

void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out)
{
    uint8_t th;

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
        export_track(out, hightest_row);
    }
    else
    {
        export_track(out, EIGHTN_START_ROW + 1u);
    }

    out->error = weighted_error(out, duty_now);
}

uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe)
{
    uint8_t rows = ti->valid_rows;
    uint8_t lost = ti->both_lost_rows;

    *severe = 0;
    /* 远端连续双丢行是八邻域搜索未到达的区域（左右搜索未相遇时
       hightest_row 保持 0，未搜索行保持初始化的极值边界），不代表
       车已冲出赛道，不计入失效统计 */
    while (rows > 0u &&
           ti->left_lost[(uint8_t)(TR_ROW(EIGHTN_START_ROW) + rows - 1u)] &&
           ti->right_lost[(uint8_t)(TR_ROW(EIGHTN_START_ROW) + rows - 1u)])
    {
        rows--;
        lost--;
    }
    if (rows < FAILSAFE_MIN_ROWS)
    {
        *severe = (uint8_t)(rows == 0u);
        return 1;
    }
    {
        uint16_t lost_pct_lhs = (uint16_t)lost * 100u;
        uint16_t rows_rhs = (uint16_t)rows;
        if (lost_pct_lhs >= rows_rhs * FAILSAFE_SEVERE_BOTH_LOST_PCT)
        {
            *severe = 1;
            return 1;
        }
        return (uint8_t)(lost_pct_lhs >= rows_rhs * FAILSAFE_MAX_BOTH_LOST_PCT);
    }
}
