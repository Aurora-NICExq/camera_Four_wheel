#include "image.h"
#include "config.h"
#include "zf_common_headfile.h"
#include <stdint.h>

volatile int16_t image_threshold = 0;
volatile uint8_t image_cross_fill = 1;
volatile uint8_t image_fill_to_look =
    EIGHTN_CROSS_FILL_TO_LOOK_DEFAULT;                     // 补线和前瞻对齐
volatile uint16_t steer_look_far = STEER_LOOK_FAR_DEFAULT; // 前瞻

#define IMG_WHITE (0xFFu)
#define IMG_BLACK (0x00u)
#define TR_ROW(ir) ((uint8_t)(IMG_H - 1u - (uint8_t)(ir)))

static uint8_t image_bin[IMG_H][IMG_W];
static uint8_t l_border[IMG_H];
static uint8_t r_border[IMG_H];

static uint16_t points_l[EIGHTN_MAX_POINTS][2];
static uint16_t points_r[EIGHTN_MAX_POINTS][2];
static uint16_t dir_l[EIGHTN_MAX_POINTS];
static uint16_t dir_r[EIGHTN_MAX_POINTS];

static uint8_t start_point_l[2];
static uint8_t start_point_r[2];
static uint16_t data_stastics_l;
static uint16_t data_stastics_r;
static uint8_t hightest_row;
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
  ti->fill_from_l = 0;
  ti->break_row_l = 0;
  ti->break_row_r = 0;
  ti->break_method_l = 0;
  ti->break_method_r = 0;
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

  // 间隔采样

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
        dir_r[r_data_statics - 1] = i;
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
}

static uint8_t fill_start_row(uint8_t break_row) {
  uint8_t from = (uint8_t)(break_row - EIGHTN_CROSS_SLOPE_NEAR);

  if (image_fill_to_look) {
    uint16_t far = steer_look_far;
    uint8_t look_ir;

    if (far > (uint16_t)STEER_LOOK_FAR_MAX) {
      far = (uint16_t)STEER_LOOK_FAR_MAX;
    }
    look_ir = (uint8_t)((uint16_t)(IMG_H - 1u) - far);
    if (look_ir < hightest_row) {
      look_ir = hightest_row;
    }
    if (look_ir < (uint8_t)EIGHTN_CROSS_FILL_ROW_MIN) {
      look_ir = (uint8_t)EIGHTN_CROSS_FILL_ROW_MIN;
    }
    if (look_ir < from) {
      from = look_ir;
    }
  }
  return from;
}

// 十字补线 — 断点检测: 方向签名(直进) → 宽松签名(斜入) → 边界外跳(兜底)

static uint8_t break_by_dir_pattern(const uint16_t *dirs,
                                    const uint16_t points[][2],
                                    uint16_t count) {
  uint16_t k;

  for (k = 1; k + 7 < count; k++) {
    if (dirs[k - 1] == 4 && dirs[k] == 4 && dirs[k + 3] == 6 &&
        dirs[k + 5] == 6 && dirs[k + 7] == 6) {
      return (uint8_t)points[k][1];
    }
  }
  return 0;
}

static uint8_t dir_is_horiz(uint16_t d) {
  return (uint8_t)(d == 0 || d == 3 || d == 4 || d == 5);
}

static uint8_t break_by_dir_flex(const uint16_t *dirs,
                                 const uint16_t points[][2], uint16_t count) {
  uint16_t k;

  for (k = 2; k + 4 < count; k++) {
    if (dir_is_horiz(dirs[k - 2]) && dir_is_horiz(dirs[k - 1]) &&
        (dirs[k] == 6 || dirs[k] == 7) && dir_is_horiz(dirs[k + 1]) &&
        dir_is_horiz(dirs[k + 2])) {
      return (uint8_t)points[k][1];
    }
  }
  return 0;
}

static uint8_t break_by_border_jump(const uint8_t *border, uint8_t outward_left,
                                    uint8_t highest) {
  int16_t r;
  uint8_t best_row = 0;
  int16_t best_score = 0;
  int16_t stop = (int16_t)highest;

  if (stop < (int16_t)(EIGHTN_CROSS_FILL_ROW_MIN + 8)) {
    stop = (int16_t)(EIGHTN_CROSS_FILL_ROW_MIN + 8);
  }

  for (r = (int16_t)EIGHTN_START_ROW; r > stop; r--) {
    int16_t dx = (int16_t)border[r] - (int16_t)border[r - 3];
    int16_t score = outward_left ? dx : (int16_t)(-dx);
    if (score > best_score) {
      best_score = score;
      best_row = (uint8_t)r;
    }
  }

  if (best_score >= (int16_t)EIGHTN_CROSS_JUMP_THRESH) {
    return best_row;
  }
  return 0;
}

static uint8_t width_stable_before(const uint8_t *border_l,
                                   const uint8_t *border_r, uint8_t break_row) {
  uint8_t lo = (uint8_t)(break_row + EIGHTN_CROSS_SLOPE_NEAR);
  uint8_t hi = (uint8_t)(lo + EIGHTN_CROSS_WIDTH_WIN);
  uint8_t r;
  int16_t widths[16];
  uint8_t n = 0;
  int16_t avg;
  int16_t sum = 0;

  if (hi >= IMG_H) {
    return 0;
  }

  for (r = lo; r < hi; r++) {
    int16_t w = (int16_t)border_r[r] - (int16_t)border_l[r];
    if (w < 8) {
      return 0;
    }
    if (n < (uint8_t)(sizeof(widths) / sizeof(widths[0]))) {
      widths[n] = w;
      n++;
    }
  }

  if (n < 4) {
    return 0;
  }

  for (r = 0; r < n; r++) {
    sum = (int16_t)(sum + widths[r]);
  }
  avg = (int16_t)(sum / (int16_t)n);

  for (r = 0; r < n; r++) {
    int16_t d = widths[r] - avg;
    if (d < 0) {
      d = (int16_t)(-d);
    }
    if (d > (int16_t)(avg / 4 + 3)) {
      return 0;
    }
  }
  return 1;
}

static uint8_t cross_bottom_ok_legacy(uint8_t bin[IMG_H][IMG_W]) {
  return (uint8_t)(bin[IMG_H - 1][EIGHTN_CROSS_CORNER_L] &&
                   bin[IMG_H - 1][EIGHTN_CROSS_CORNER_R]);
}

static uint8_t cross_bottom_ok(uint8_t bin[IMG_H][IMG_W]) {
  uint16_t c;
  uint8_t left;
  uint8_t right;
  uint8_t center_track = 0;

  if (cross_bottom_ok_legacy(bin)) {
    return 1;
  }

  left = bin[IMG_H - 1][EIGHTN_CROSS_CORNER_L];
  right = bin[IMG_H - 1][EIGHTN_CROSS_CORNER_R];
  for (c = (uint16_t)(IMG_CENTER - 20); c < (uint16_t)(IMG_CENTER + 20); c++) {
    if (bin[IMG_H - 1][c]) {
      center_track = 1;
      break;
    }
  }
  return (uint8_t)((left || right) && center_track);
}

static void pick_break(const uint16_t *dirs, const uint16_t points[][2],
                       uint16_t count, const uint8_t *border,
                       uint8_t outward_left, uint8_t *row_out,
                       uint8_t *method_out) {
  uint8_t row;

  row = break_by_dir_pattern(dirs, points, count);
  if (row) {
    *row_out = row;
    *method_out = 1;
    return;
  }

  row = break_by_dir_flex(dirs, points, count);
  if (row) {
    *row_out = row;
    *method_out = 2;
    return;
  }

  row = break_by_border_jump(border, outward_left, hightest_row);
  if (row) {
    *row_out = row;
    *method_out = 3;
    return;
  }

  *row_out = 0;
  *method_out = 0;
}

static void apply_side_fill(uint8_t break_row, uint8_t left, uint8_t fill_from,
                            track_info_t *ti) {
  uint8_t *border = left ? l_border : r_border;
  uint8_t start = (uint8_t)limit_a_b(
      (int16_t)break_row - (int16_t)EIGHTN_CROSS_SLOPE_BACK, 0, IMG_H - 1);
  uint8_t end = (uint8_t)limit_a_b(
      (int16_t)break_row - (int16_t)EIGHTN_CROSS_SLOPE_NEAR, 0, IMG_H - 1);
  float slope_rate;
  float intercept;
  uint8_t i;

  calculate_s_i(start, end, border, &slope_rate, &intercept);
  intercept = (float)border[end] - slope_rate * (float)end;

  for (i = fill_from; i < (uint8_t)(IMG_H - 1); i++) {
    float fv = slope_rate * (float)i + intercept;
    int16_t v = (int16_t)(fv >= 0.0f ? fv + 0.5f : fv - 0.5f);
    border[i] = (uint8_t)limit_a_b(v, EIGHTN_BORDER_MIN, EIGHTN_BORDER_MAX);
    mark_cross_fill_rows(i, i, ti);
  }
}

static void cross_fill(uint8_t bin[IMG_H][IMG_W], track_info_t *ti) {
  uint8_t break_num_l = 0;
  uint8_t break_num_r = 0;
  uint8_t method_l = 0;
  uint8_t method_r = 0;
  uint8_t fill_from_l;
  uint8_t fill_from_r;
  uint8_t fill_from;
  uint8_t both_pattern;
  uint8_t row_tol;
  int16_t row_diff;

  pick_break(dir_l, points_l, data_stastics_l, l_border, 1, &break_num_l,
             &method_l);
  pick_break(dir_r, points_r, data_stastics_r, r_border, 0, &break_num_r,
             &method_r);

  ti->break_row_l = break_num_l;
  ti->break_row_r = break_num_r;
  ti->break_method_l = method_l;
  ti->break_method_r = method_r;

  if (!break_num_l || !break_num_r) {
    return;
  }

  // 拟合窗 [break-BACK, break-NEAR) 必须整段落在图像内

  if (break_num_l <= EIGHTN_CROSS_SLOPE_BACK ||
      break_num_r <= EIGHTN_CROSS_SLOPE_BACK) {
    return;
  }

  both_pattern = (uint8_t)(method_l == 1 && method_r == 1);
  row_tol = both_pattern ? EIGHTN_CROSS_BREAK_ROW_TOL_PATTERN
                         : EIGHTN_CROSS_BREAK_ROW_TOL;
  row_diff = (int16_t)break_num_l - (int16_t)break_num_r;
  if (row_diff < 0) {
    row_diff = (int16_t)(-row_diff);
  }
  if ((uint8_t)row_diff > row_tol) {
    return;
  }

  if (!both_pattern) {
    if (break_num_l <= (uint8_t)(hightest_row + 2) ||
        break_num_r <= (uint8_t)(hightest_row + 2)) {
      return;
    }
    if (!width_stable_before(l_border, r_border,
                             break_num_l > break_num_r ? break_num_l
                                                       : break_num_r)) {
      return;
    }
  } else {
    if (break_num_l <= hightest_row || break_num_r <= hightest_row) {
      return;
    }
  }

  if (!cross_bottom_ok(bin)) {
    return;
  }

  fill_from_l = fill_start_row(break_num_l);
  fill_from_r = fill_start_row(break_num_r);
  fill_from = fill_from_l < fill_from_r ? fill_from_l : fill_from_r;
  ti->fill_from_l = fill_from;

  apply_side_fill(break_num_l, 1, fill_from, ti);
  apply_side_fill(break_num_r, 0, fill_from, ti);
  ti->cross_valid = 1;
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

static int16_t look_ahead_error(const track_info_t *ti, uint8_t *look_n_out) {
  const uint8_t span = (uint8_t)STEER_LOOK_SPAN; // 取20行前瞻
  int32_t acc = 0;
  uint8_t n = 0;
  uint8_t r;
  uint16_t far = steer_look_far;

  if (far > (uint16_t)STEER_LOOK_FAR_MAX) {
    far = (uint16_t)STEER_LOOK_FAR_MAX;
  }
  if (far <= (uint16_t)span) {
    far = (uint16_t)span + 1u;
  }
  r = (uint8_t)far;

  while (r > 0u && n < span) {
    uint8_t tr;

    r--;
    tr = (uint8_t)(TR_ROW(EIGHTN_START_ROW) + r);
    if (ti->left_lost[tr] && ti->right_lost[tr]) { // 丢线不计入
      continue;
    }
    acc += (int16_t)ti->mid[tr] - IMG_CENTER;
    n++;
  }

  *look_n_out = n;
  if (n == 0u) {
    if (g_hold_frames < ERR_HOLD_MAX_FRAMES) {
      g_hold_frames++;
    } else {
      g_err_hold = (int16_t)((g_err_hold * 3) / 4);
    }
    return g_err_hold;
  }
  g_hold_frames = 0;
  g_err_hold = (int16_t)(acc / (int32_t)n);
  return g_err_hold; // 平均误差计算
}

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out) {
  uint8_t th;
  uint8_t look_n;

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

  out->error = look_ahead_error(out, &look_n);
  out->look_rows = look_n;
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
