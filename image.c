#include "image.h"
#include "config.h"
#include "zf_common_headfile.h"
#include <stdint.h>

volatile int16_t image_threshold = 0;
volatile uint8_t image_cross_fill = 1;
volatile uint16_t steer_look_far = STEER_LOOK_FAR_DEFAULT; // 前瞻

#define IMG_WHITE (0xFFu)
#define IMG_BLACK (0x00u)
#define TR_ROW(ir) ((uint8_t)(IMG_H - 1u - (uint8_t)(ir)))

static uint8_t image_bin[IMG_H][IMG_W];
static uint8_t l_border[IMG_H];
static uint8_t r_border[IMG_H];

static uint16_t points_l[EIGHTN_MAX_POINTS][2];
static uint16_t points_r[EIGHTN_MAX_POINTS][2];
/* dir_l 只在 search_l_r 内部用于左右爬线同步;
   dir_r 随向量法拐点检测一并删除(原来只有十字检测读它) */
static uint16_t dir_l[EIGHTN_MAX_POINTS];

static uint8_t start_point_l[2];
static uint8_t start_point_r[2];
static uint16_t data_stastics_l;
static uint16_t data_stastics_r;
static uint8_t hightest_row;
static uint8_t cross_break_l;
static uint8_t cross_break_r;
static uint8_t cross_flag;
static int16_t g_err_hold;
static uint8_t g_hold_frames;

static int my_abs(int v) { return (v >= 0) ? v : -v; }

static int16_t limit_a_b(int16_t x, int16_t a, int16_t b) {
  if (x < a)
    return a;
  if (x > b)
    return b;
  return x;
}

static void init_cross_meta(track_info_t *ti) {
  uint8_t r;
  ti->cross_valid = 0;
  ti->cross_lo = 0;
  ti->cross_hi = 0;
  ti->inflect_row = 0xFF;
  for (r = 0; r < IMG_H; r++) {
    ti->cross_filled[r] = 0;
  }
}

static uint8_t clamp_u8(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo)
    return (uint8_t)lo;
  if (v > hi)
    return (uint8_t)hi;
  return (uint8_t)v;
}

// 大津法

static uint8_t otsu_threshold(const uint8_t img[IMG_H][IMG_W]) {
  uint32_t hist[256] = {0};
  uint32_t total = 0;
  uint16_t r, c, i;

  for (r = 0; r < IMG_H; r += OTSU_ROW_STEP) {
    for (c = 0; c < IMG_W; c += OTSU_COL_STEP) {
      hist[img[r][c]]++;
      total++;
    }
  }

  uint64_t sum_all = 0;
  for (i = 0; i < 256; i++) {
    sum_all += (uint64_t)i * hist[i];
  }

  uint64_t best_var = 0;
  uint16_t best_th = FIXED_THRESHOLD;
  uint32_t w0 = 0;
  uint64_t sum0 = 0;

  for (i = 0; i < 256; i++) {
    w0 += hist[i];
    if (w0 == 0)
      continue;
    uint32_t w1 = total - w0;
    if (w1 == 0)
      break;
    sum0 += (uint64_t)i * hist[i];

    int64_t diff = (int64_t)(sum0 * w1) - (int64_t)((sum_all - sum0) * w0);
    uint64_t d2 = (uint64_t)((diff < 0) ? -diff : diff);
    uint64_t var = (d2 / w0) * (d2 / w1);
    if (var > best_var) {
      best_var = var;
      best_th = i;
    }
  }

  if (best_th < OTSU_THRESHOLD_MIN)
    best_th = OTSU_THRESHOLD_MIN;
  if (best_th > OTSU_THRESHOLD_MAX)
    best_th = OTSU_THRESHOLD_MAX;
  return (uint8_t)best_th;
}

// 二值化处理

static void binarize(const uint8_t img[IMG_H][IMG_W], uint8_t th) {
  uint16_t r, c;
  for (r = 0; r < IMG_H; r++) {
    for (c = 0; c < IMG_W; c++) {
      image_bin[r][c] = (img[r][c] >= th) ? IMG_WHITE : IMG_BLACK;
    }
  }
}

static void image_filter(uint8_t bin[IMG_H][IMG_W]) {
  uint16_t r, c;
  for (r = 1; r < IMG_H - 1; r++) {
    for (c = 1; c < IMG_W - 1; c++) {
      uint32_t num = bin[r - 1][c - 1] + bin[r - 1][c] + bin[r - 1][c + 1] +
                     bin[r][c - 1] + bin[r][c + 1] + bin[r + 1][c - 1] +
                     bin[r + 1][c] + bin[r + 1][c + 1];

      if (num >= (uint32_t)EIGHTN_FILTER_SUM_MAX && bin[r][c] == IMG_BLACK) {
        bin[r][c] = IMG_WHITE;
      }
      if (num <= (uint32_t)EIGHTN_FILTER_SUM_MIN && bin[r][c] == IMG_WHITE) {
        bin[r][c] = IMG_BLACK;
      }
    }
  }
}

// 降噪保护

static void image_draw_rectan(uint8_t bin[IMG_H][IMG_W]) {
  uint16_t r, c;
  for (r = 0; r < IMG_H; r++) {
    bin[r][0] = IMG_BLACK;
    bin[r][1] = IMG_BLACK;
    bin[r][IMG_W - 1] = IMG_BLACK;
    bin[r][IMG_W - 2] = IMG_BLACK;
  }
  for (c = 0; c < IMG_W; c++) {
    bin[0][c] = IMG_BLACK;
    bin[1][c] = IMG_BLACK;
  }
}

// 起点

static uint8_t get_start_point(uint8_t start_row) {
  uint16_t i;
  uint8_t l_found = 0;
  uint8_t r_found = 0;

  start_point_l[0] = 0;
  start_point_l[1] = 0;
  start_point_r[0] = 0;
  start_point_r[1] = 0;

  // 从中线开始左右找

  for (i = IMG_W / 2; i > EIGHTN_BORDER_MIN; i--) {
    start_point_l[0] = (uint8_t)i;
    start_point_l[1] = start_row;
    if (image_bin[start_row][i] == IMG_WHITE &&
        image_bin[start_row][i - 1] == IMG_BLACK) {
      l_found = 1;
      break;
    }
  }

  for (i = IMG_W / 2; i < EIGHTN_BORDER_MAX; i++) {
    start_point_r[0] = (uint8_t)i;
    start_point_r[1] = start_row;
    if (image_bin[start_row][i] == IMG_WHITE &&
        image_bin[start_row][i + 1] == IMG_BLACK) {
      r_found = 1;
      break;
    }
  }

  return (uint8_t)(l_found && r_found);
}

static void search_l_r(uint16_t break_flag, uint8_t bin[IMG_H][IMG_W],
                       uint16_t *l_stastic, uint16_t *r_stastic,
                       uint8_t l_start_x, uint8_t l_start_y, uint8_t r_start_x,
                       uint8_t r_start_y, uint8_t *hightest) {
  static const int8_t seeds_l[8][2] = {{0, 1},  {-1, 1}, {-1, 0}, {-1, -1},
                                       {0, -1}, {1, -1}, {1, 0},  {1, 1}};
  static const int8_t seeds_r[8][2] = {{0, 1},  {1, 1},   {1, 0},  {1, -1},
                                       {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

  uint8_t search_filds_l[8][2];
  uint8_t search_filds_r[8][2];
  uint8_t temp_l[8][2];
  uint8_t temp_r[8][2];
  uint8_t center_point_l[2];
  uint8_t center_point_r[2];
  uint8_t index_l;
  uint8_t index_r;
  uint8_t i;
  uint8_t j;
  uint16_t l_data_statics = *l_stastic;
  uint16_t r_data_statics = *r_stastic;

  center_point_l[0] = l_start_x;
  center_point_l[1] = l_start_y;
  center_point_r[0] = r_start_x;
  center_point_r[1] = r_start_y;

  while (break_flag--) {
    for (i = 0; i < 8; i++) {
      search_filds_l[i][0] = (uint8_t)(center_point_l[0] + seeds_l[i][0]);
      search_filds_l[i][1] = (uint8_t)(center_point_l[1] + seeds_l[i][1]);
    }
    points_l[l_data_statics][0] = center_point_l[0];
    points_l[l_data_statics][1] = center_point_l[1];
    l_data_statics++;

    for (i = 0; i < 8; i++) {
      search_filds_r[i][0] = (uint8_t)(center_point_r[0] + seeds_r[i][0]);
      search_filds_r[i][1] = (uint8_t)(center_point_r[1] + seeds_r[i][1]);
    }
    points_r[r_data_statics][0] = center_point_r[0];
    points_r[r_data_statics][1] = center_point_r[1];

    index_l = 0;
    for (i = 0; i < 8; i++) {
      temp_l[i][0] = 0;
      temp_l[i][1] = 0;
    }

    for (i = 0; i < 8; i++) {
      if (bin[search_filds_l[i][1]][search_filds_l[i][0]] == IMG_BLACK &&
          bin[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] ==
              IMG_WHITE) {
        temp_l[index_l][0] = search_filds_l[i][0];
        temp_l[index_l][1] = search_filds_l[i][1];
        dir_l[l_data_statics - 1] = i;
        index_l++;
      }

      if (index_l) {
        center_point_l[0] = temp_l[0][0];
        center_point_l[1] = temp_l[0][1];
        for (j = 0; j < index_l; j++) {
          if (center_point_l[1] > temp_l[j][1]) {
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
         points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1])) {
      break;
    }

    if (my_abs((int)points_r[r_data_statics][0] -
               (int)points_l[l_data_statics - 1][0]) < EIGHTN_MEET_DIST &&
        my_abs((int)points_r[r_data_statics][1] -
               (int)points_l[l_data_statics - 1][1]) < EIGHTN_MEET_DIST) {
      *hightest = (uint8_t)((points_r[r_data_statics][1] +
                             points_l[l_data_statics - 1][1]) >>
                            1);
      break;
    }

    if (points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]) {
      continue;
    }

    if (dir_l[l_data_statics - 1] == 7 &&
        points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]) {
      center_point_l[0] = (uint8_t)points_l[l_data_statics - 1][0];
      center_point_l[1] = (uint8_t)points_l[l_data_statics - 1][1];
      l_data_statics--;
    }
    r_data_statics++;

    index_r = 0;
    for (i = 0; i < 8; i++) {
      temp_r[i][0] = 0;
      temp_r[i][1] = 0;
    }

    for (i = 0; i < 8; i++) {
      if (bin[search_filds_r[i][1]][search_filds_r[i][0]] == IMG_BLACK &&
          bin[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] ==
              IMG_WHITE) {
        temp_r[index_r][0] = search_filds_r[i][0];
        temp_r[index_r][1] = search_filds_r[i][1];
        index_r++;
      }

      if (index_r) {
        center_point_r[0] = temp_r[0][0];
        center_point_r[1] = temp_r[0][1];
        for (j = 0; j < index_r; j++) {
          if (center_point_r[1] > temp_r[j][1]) {
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

static void get_left(uint16_t total_l) {
  uint16_t j;
  int16_t h = (int16_t)EIGHTN_START_ROW;
  uint8_t i;

  for (i = 0; i < IMG_H; i++) {
    l_border[i] = EIGHTN_BORDER_MIN;
  }

  for (j = 0; j < total_l; j++) {
    if (points_l[j][1] == (uint16_t)h) {
      l_border[h] = (uint8_t)(points_l[j][0] + 1u);
    } else {
      continue;
    }
    h--;
    if (h <= 0) {
      break;
    }
  }
}

static void get_right(uint16_t total_r) {
  uint16_t j;
  int16_t h = (int16_t)EIGHTN_START_ROW;
  uint8_t i;

  for (i = 0; i < IMG_H; i++) {
    r_border[i] = EIGHTN_BORDER_MAX;
  }

  for (j = 0; j < total_r; j++) {
    if (points_r[j][1] == (uint16_t)h) {
      r_border[h] = (uint8_t)(points_r[j][0] - 1u);
    } else {
      continue;
    }
    h--;
    if (h <= 0) {
      break;
    }
  }
}

// 十字补线

static float slope_calculate(uint8_t begin, uint8_t end,
                             const uint8_t *border) {
  float xsum = 0.0f;
  float ysum = 0.0f;
  float xysum = 0.0f;
  float x2sum = 0.0f;
  int16_t i;
  float result = 0.0f;
  static float result_last;

  for (i = (int16_t)begin; i < (int16_t)end; i++) {
    xsum += (float)i;
    ysum += (float)border[i];
    xysum += (float)i * (float)border[i];
    x2sum += (float)i * (float)i;
  }

  if (((float)(end - begin) * x2sum - xsum * xsum) != 0.0f) {
    result = (((float)(end - begin) * xysum - xsum * ysum) /
              ((float)(end - begin) * x2sum - xsum * xsum));
    result_last = result;
  } else {
    result = result_last;
  }
  return result;
}

static void calculate_s_i(uint8_t start, uint8_t end, uint8_t *border,
                          float *slope_rate, float *intercept) {
  uint16_t i;
  uint16_t num = 0;
  uint16_t xsum = 0;
  uint16_t ysum = 0;
  float x_average;
  float y_average;

  for (i = start; i < end; i++) {
    xsum += i;
    ysum += border[i];
    num++;
  }

  if (num) {
    x_average = (float)(xsum / num);
    y_average = (float)(ysum / num);
  } else {
    x_average = 0.0f;
    y_average = 0.0f;
  }

  // 直线拟合

  *slope_rate = slope_calculate(start, end, border);
  *intercept = y_average - (*slope_rate) * x_average;
}

static void mark_cross_fill_rows(uint8_t y_lo, uint8_t y_hi, track_info_t *ti) {
  uint8_t a1 = y_lo;
  uint8_t a2 = y_hi;
  uint8_t i;

  if (a1 > a2) {
    uint8_t t = a1;
    a1 = a2;
    a2 = t;
  }

  for (i = a1; i <= a2; i++) {
    uint8_t tr = TR_ROW(i);
    if (tr < IMG_H) {
      ti->cross_filled[tr] = 1;
    }
  }

  if (ti->cross_lo == 0 && ti->cross_hi == 0) {
    ti->cross_lo = TR_ROW(a2);
    ti->cross_hi = (uint8_t)(TR_ROW(a1) + 1u);
  }
}

/* 向量法找上拐点(车已进入十字时,画面里还看得见的那两个角)。

   爬线从画面最下面沿人工黑框往上走,到十字横向开口的上沿转向横里,
   走到赛道走廊边缘再转竖直往上。要找的就是第二个转折:
     进入向量 a 几乎水平(沿开口上沿走) → 离开向量 b 明显向上(转进远端走廊)
   第一个转折(黑框处由竖转横)进入段是竖直的,会被 |ay|<=FLAT 挡掉,
   这一点比原来的 dir 序列判据更准 —— 旧判据抓的是黑框那个转折。

   x_dir: 左边线横向段朝右传 +1,右边线横向段朝左传 -1。
   返回角点所在图像行(>SLOPE_BACK),0 表示没找到。 */
static uint8_t find_up_corner(const uint16_t pts[][2], uint16_t total,
                              int8_t x_dir) {
  const uint16_t k = (uint16_t)EIGHTN_CROSS_VEC_K;
  const int16_t flat = (int16_t)EIGHTN_CROSS_VEC_FLAT;
  uint16_t i;

  for (i = 0; i + 2u * k < total; i++) {
    int16_t ax = (int16_t)pts[i + k][0] - (int16_t)pts[i][0];
    int16_t ay = (int16_t)pts[i + k][1] - (int16_t)pts[i][1];
    int16_t bx = (int16_t)pts[i + 2u * k][0] - (int16_t)pts[i + k][0];
    int16_t by = (int16_t)pts[i + 2u * k][1] - (int16_t)pts[i + k][1];

    /* 角点太靠远端,下游的拟合窗口 [break-SLOPE_BACK, break) 放不下,
       跳过继续找,而不是整帧放弃 */
    if (pts[i + k][1] <= (uint16_t)EIGHTN_CROSS_SLOPE_BACK) {
      continue;
    }
    /* 转角不超过 90 度:折返说明是噪声毛刺或爬线绕回来了 */
    if ((int32_t)ax * bx + (int32_t)ay * by < 0) {
      continue;
    }
    /* 进入段必须几乎水平 */
    if (ay > flat || ay < -flat) {
      continue;
    }
    /* 离开段必须明显向上(图像行号减小) */
    if (by > -flat) {
      continue;
    }
    /* 横向段的朝向,以及整体走向必须向上 */
    if (x_dir > 0) {
      if (ax <= 0 || (ax + bx) < 0) {
        continue;
      }
    } else {
      if (ax >= 0 || (ax + bx) > 0) {
        continue;
      }
    }
    if ((ay + by) > 0) {
      continue;
    }
    return (uint8_t)pts[i + k][1]; /* 角点是中间那个点 */
  }
  return 0;
}

static void cross_fill(uint8_t bin[IMG_H][IMG_W], track_info_t *ti) {
  uint16_t i;
  uint8_t start;
  uint8_t end;
  float slope_rate = 0.0f;
  float intercept = 0.0f;
  uint8_t fill_from;

  cross_break_l = 0;
  cross_break_r = 0;
  cross_flag = 0;

  /* 左边线的横向段朝右走,右边线的横向段朝左走 */
  cross_break_l = find_up_corner(points_l, data_stastics_l, +1);
  cross_break_r = find_up_corner(points_r, data_stastics_r, -1);

  if (!cross_break_l || !cross_break_r) {
    return;
  }

  if (!bin[IMG_H - 1][EIGHTN_CROSS_CORNER_L] ||
      !bin[IMG_H - 1][EIGHTN_CROSS_CORNER_R]) {
    return;
  }

  /* 拟合窗口需完整落在图像内(break-SLOPE_BACK 不得下溢,否则越界拟合) */
  if (cross_break_l <= EIGHTN_CROSS_SLOPE_BACK ||
      cross_break_r <= EIGHTN_CROSS_SLOPE_BACK) {
    return;
  }

  /* 真十字:左右上拐点行号接近,且拐点下方开口接近全宽。
     弯道的拐点同样能通过向量法的几何判据(横向进、竖直出),
     但内侧边界仍在、宽度不足,误补线会把弯道中线拉直、掏空转向误差 */
  if (my_abs((int)cross_break_l - (int)cross_break_r) > EIGHTN_CROSS_BREAK_DROW) {
    return;
  }
  {
    uint8_t base =
        (cross_break_l > cross_break_r) ? cross_break_l : cross_break_r;
    uint16_t row;
    uint8_t samples = 0;
    uint8_t open_cnt = 0;
    for (row = (uint16_t)base + 2u;
         row < (uint16_t)base + 12u && row <= (uint16_t)EIGHTN_CROSS_OPEN_ROW_MAX;
         row++) {
      samples++;
      if (((int16_t)r_border[row] - (int16_t)l_border[row]) >=
          EIGHTN_CROSS_OPEN_WIDTH) {
        open_cnt++;
      }
    }
    if (samples >= 4u && ((uint16_t)open_cnt * 3u) < ((uint16_t)samples * 2u)) {
      return;
    }
  }

  cross_flag = 1;
  fill_from = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_NEAR);

  start = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_BACK);
  start = (uint8_t)limit_a_b((int16_t)start, 0, IMG_H - 1);
  end = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_NEAR);
  calculate_s_i(start, end, l_border, &slope_rate, &intercept);
  for (i = fill_from; i < (uint16_t)(IMG_H - 1); i++) {
    int16_t v = (int16_t)(slope_rate * (float)i + intercept);
    l_border[i] = (uint8_t)limit_a_b(v, EIGHTN_BORDER_MIN, EIGHTN_BORDER_MAX);
    mark_cross_fill_rows((uint8_t)i, (uint8_t)i, ti);
  }

  start = (uint8_t)(cross_break_r - EIGHTN_CROSS_SLOPE_BACK);
  start = (uint8_t)limit_a_b((int16_t)start, 0, IMG_H - 1);
  end = (uint8_t)(cross_break_r - EIGHTN_CROSS_SLOPE_NEAR);
  calculate_s_i(start, end, r_border, &slope_rate, &intercept);
  fill_from = (uint8_t)(cross_break_r - EIGHTN_CROSS_SLOPE_NEAR);
  for (i = fill_from; i < (uint16_t)(IMG_H - 1); i++) {
    int16_t v = (int16_t)(slope_rate * (float)i + intercept);
    r_border[i] = (uint8_t)limit_a_b(v, EIGHTN_BORDER_MIN, EIGHTN_BORDER_MAX);
    mark_cross_fill_rows((uint8_t)i, (uint8_t)i, ti);
  }

  ti->cross_valid = 1;
  ti->inflect_row = TR_ROW(cross_break_l);
}

static void export_track(track_info_t *ti, uint8_t hightest) {
  uint8_t ir;
  uint8_t tr;
  uint8_t both_lost = 0;
  uint8_t lo = hightest;
  uint8_t hi = EIGHTN_START_ROW;

  for (tr = 0; tr < IMG_H; tr++) {
    ti->left[tr] = 0;
    ti->right[tr] = (uint8_t)(IMG_W - 1);
    ti->mid[tr] = IMG_CENTER;
    ti->left_lost[tr] = 1;
    ti->right_lost[tr] = 1;
  }

  if (lo > hi) {
    ti->both_lost_rows = 0;
    return;
  }

  for (ir = lo; ir <= hi; ir++) {
    tr = TR_ROW(ir);
    ti->left[tr] = clamp_u8(l_border[ir], 0, IMG_W - 1);
    ti->right[tr] = clamp_u8(r_border[ir], 0, IMG_W - 1);
    ti->mid[tr] =
        (uint8_t)(((uint16_t)ti->left[tr] + (uint16_t)ti->right[tr]) / 2u);
    ti->left_lost[tr] = (uint8_t)(l_border[ir] <= (EIGHTN_BORDER_MIN +
                                                   EIGHTN_EDGE_LOST_MARGIN));
    ti->right_lost[tr] = (uint8_t)(r_border[ir] >= (EIGHTN_BORDER_MAX -
                                                    EIGHTN_EDGE_LOST_MARGIN));
    if (ti->left_lost[tr] && ti->right_lost[tr]) {
      both_lost++;
    }
  }

  ti->both_lost_rows = both_lost;
}

// 前瞻代码使用，超念是对的

/* 单行前瞻:Look Far 即瞄准行(tr);双边丢线则向近端滑,*aim_tr_out=0 表示无有效行。 */
static int16_t look_ahead_error(const track_info_t *ti, uint8_t *aim_tr_out) {
  uint8_t r;
  uint16_t far = steer_look_far;

  if (far > (uint16_t)STEER_LOOK_FAR_MAX) {
    far = (uint16_t)STEER_LOOK_FAR_MAX;
  }
  if (far < 1u) {
    far = 1u;
  }
  r = (uint8_t)far;

  while (r > 0u) {
    uint8_t tr;

    r--;
    tr = (uint8_t)(TR_ROW(EIGHTN_START_ROW) + r);
    if (ti->left_lost[tr] && ti->right_lost[tr]) {
      continue;
    }
    *aim_tr_out = tr;
    g_hold_frames = 0;
    g_err_hold = (int16_t)ti->mid[tr] - IMG_CENTER;
    return g_err_hold;
  }

  *aim_tr_out = 0u;
  if (g_hold_frames < ERR_HOLD_MAX_FRAMES) {
    g_hold_frames++;
  } else {
    g_err_hold = (int16_t)((g_err_hold * 3) / 4);
  }
  return g_err_hold;
}

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out) {
  uint8_t th;
  uint8_t aim_tr;

  init_cross_meta(out);
  hightest_row = 0;
  data_stastics_l = 0;
  data_stastics_r = 0;

  th = (image_threshold > 0) ? (uint8_t)image_threshold : otsu_threshold(img);
  out->threshold = th;

  binarize(img, th);
  image_filter(image_bin);
  image_draw_rectan(image_bin);

  if (get_start_point(EIGHTN_START_ROW)) {
    search_l_r((uint16_t)EIGHTN_MAX_POINTS, image_bin, &data_stastics_l,
               &data_stastics_r, start_point_l[0], start_point_l[1],
               start_point_r[0], start_point_r[1], &hightest_row);
    get_left(data_stastics_l);
    get_right(data_stastics_r);
    if (image_cross_fill) {
      cross_fill(image_bin, out);
    }
    export_track(out, hightest_row);
  } else {
    export_track(out, EIGHTN_START_ROW + 1u);
  }

  out->error = look_ahead_error(out, &aim_tr);
  out->aim_row = aim_tr;
  out->err_hold = g_hold_frames;
}

static void debug_draw_seg(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                           uint16_t color) {
  if (x0 == x1 && y0 == y1) {
    ips200_draw_point(x0, y0, color);
  } else {
    ips200_draw_line(x0, y0, x1, y1, color);
  }
}

void image_debug_show(const track_info_t *ti) {
  uint8_t tr;
  uint8_t tr0 = (uint8_t)TR_ROW(EIGHTN_START_ROW);
  uint8_t any = 0;

  ips200_show_gray_image(0, 0, (const uint8 *)image_bin, IMG_W, IMG_H, IMG_W,
                         IMG_H, 128);

  for (tr = (uint8_t)(tr0 + 1u); tr < IMG_H; tr++) {
    uint8_t prev = (uint8_t)(tr - 1u);
    uint16_t y0 = (uint16_t)(IMG_H - 1u - prev);
    uint16_t y1 = (uint16_t)(IMG_H - 1u - tr);
    uint8_t filled0 = ti->cross_filled[prev];
    uint8_t filled1 = ti->cross_filled[tr];

    if (!ti->left_lost[prev] && !ti->left_lost[tr]) {
      debug_draw_seg(ti->left[prev], y0, ti->left[tr], y1,
                     (filled0 || filled1) ? RGB565_YELLOW : RGB565_BLUE);
      any = 1;
    }
    if (!ti->right_lost[prev] && !ti->right_lost[tr]) {
      debug_draw_seg(ti->right[prev], y0, ti->right[tr], y1,
                     (filled0 || filled1) ? RGB565_YELLOW : RGB565_RED);
      any = 1;
    }
    if (!ti->left_lost[prev] && !ti->right_lost[prev] && !ti->left_lost[tr] &&
        !ti->right_lost[tr]) {
      debug_draw_seg(ti->mid[prev], y0, ti->mid[tr], y1, RGB565_GREEN);
      any = 1;
    }
  }

  if (!any && (!ti->left_lost[tr0] || !ti->right_lost[tr0])) {
    uint16_t y = (uint16_t)(IMG_H - 1u - tr0);
    if (!ti->left_lost[tr0]) {
      ips200_draw_point(ti->left[tr0], y,
                        ti->cross_filled[tr0] ? RGB565_YELLOW : RGB565_BLUE);
    }
    if (!ti->right_lost[tr0]) {
      ips200_draw_point(ti->right[tr0], y,
                        ti->cross_filled[tr0] ? RGB565_YELLOW : RGB565_RED);
    }
    if (!ti->left_lost[tr0] && !ti->right_lost[tr0]) {
      ips200_draw_point(ti->mid[tr0], y, RGB565_GREEN);
    }
  }
}
