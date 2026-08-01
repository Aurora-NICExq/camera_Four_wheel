#include "image.h"
#include "config.h"
#include "zf_common_headfile.h"
#include <stdint.h>

volatile int16_t image_threshold = 0;
volatile uint8_t image_cross_fill = 1;
volatile uint16_t steer_look_far = STEER_LOOK_FAR_DEFAULT;

#define IMG_WHITE (0xFFu)
#define IMG_BLACK (0x00u)
#define TR_ROW(ir) ((uint8_t)(IMG_H - 1u - (uint8_t)(ir)))

/* 参考库 RoadType,十字补线门控用 */
typedef enum {
  LWC_ROAD_STRAIGHT = 0,
  LWC_ROAD_LEFT_TURN,
  LWC_ROAD_RIGHT_TURN,
  LWC_ROAD_CROSSING,
  LWC_ROAD_LEFT_HUANDAO,
  LWC_ROAD_RIGHT_HUANDAO,
  LWC_ROAD_RAMP,
  LWC_ROAD_BANMAXIAN
} lwc_road_type_e;

static uint8_t image_bin[IMG_H][IMG_W];

/* 行号 ir:0=画面顶(远),IMG_H-1=画面底(近)。导出 track_info_t 时用 TR_ROW() */
static uint8_t l_border[IMG_H];
static uint8_t r_border[IMG_H];
static uint8_t l_lost[IMG_H];
static uint8_t r_lost[IMG_H];
static uint8_t road_wide[IMG_H];

static int white_col[IMG_W];

static uint8_t lwc_len;
static uint8_t lwc_col;
static uint8_t last_lwc_len;
static uint8_t last_lwc_col = IMG_CENTER; /* 参考库 main 初始化种子列 94 */
static uint8_t search_stop_line;
static uint8_t stop_row;

static uint16_t left_lost_time;
static uint16_t right_lost_time;
static uint16_t both_lost_time;
static uint8_t boundry_start_left;
static uint8_t boundry_start_right;

static lwc_road_type_e road_type;

static int16_t up_find_l;
static int16_t up_find_r;
static int16_t down_find_l;
static int16_t down_find_r;
static int16_t last_up_find_l;
static int16_t last_up_find_r;

static int16_t g_err_hold;
static uint8_t g_hold_frames;

static void outer_analyse(void);

static int my_abs(int v) { return (v >= 0) ? v : -v; }

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

      if (num >= (uint32_t)LWC_FILTER_SUM_MAX && bin[r][c] == IMG_BLACK) {
        bin[r][c] = IMG_WHITE;
      }
      if (num <= (uint32_t)LWC_FILTER_SUM_MIN && bin[r][c] == IMG_WHITE) {
        bin[r][c] = IMG_BLACK;
      }
    }
  }
}

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

/* ================= 参考库 Center_line_deal_plus ================= */

static void center_line_deal_plus(uint8_t start_column, uint8_t end_column) {
  uint8_t j;
  int16_t ir;
  int16_t row;
  int16_t col;
  int left_border;
  int right_border;

begin:

  for (ir = 0; ir < (int16_t)IMG_H - 1; ir++) {
    l_border[ir] = 0;
    r_border[ir] = 0;
    l_lost[ir] = 0;
    r_lost[ir] = 0;
  }

  left_lost_time = 0;
  right_lost_time = 0;
  both_lost_time = 0;
  boundry_start_left = 0;
  boundry_start_right = 0;

  for (j = 0; j < IMG_W; j++) {
    white_col[j] = 0;
  }

  /* 参考库:自 IMAGE_HEIGHT-3 向上数连续赛道像素(参考库 BLACK_POINT=赛道) */
  for (j = start_column; j <= end_column; j++) {
    for (ir = (int16_t)IMG_H - 3; ir >= 0; ir--) {
      if (image_bin[ir][j] == IMG_WHITE) {
        white_col[j]++;
        if (white_col[j] >= (int)IMG_H) {
          break;
        }
      } else {
        break;
      }
    }
    /* 参考库:底行两格都是赛道则整列作废(极性已适配) */
    if (image_bin[IMG_H - 1][j] == IMG_WHITE &&
        image_bin[IMG_H - 2][j] == IMG_WHITE) {
      white_col[j] = 0;
    }
  }

  last_lwc_len = lwc_len;
  last_lwc_col = lwc_col;
  lwc_len = 0;
  lwc_col = IMG_CENTER;

  for (j = start_column; j <= end_column; j++) {
    if ((uint8_t)white_col[j] > lwc_len) {
      lwc_len = (uint8_t)white_col[j];
      lwc_col = j;
    }
  }

  search_stop_line = lwc_len;
  stop_row = (uint8_t)(LWC_SCAN_START_ROW + 1u - search_stop_line);

  if (my_abs((int)lwc_col - (int)last_lwc_col) >= LWC_COL_JUMP_MAX) {
    lwc_len = last_lwc_len;
    lwc_col = last_lwc_col;
    search_stop_line = lwc_len;
    stop_row = (uint8_t)(LWC_SCAN_START_ROW + 1u - search_stop_line);
  }

  if (search_stop_line > 0u) {
  for (row = (int16_t)LWC_SCAN_START_ROW;
       row >= (int16_t)(IMG_H - search_stop_line); row--) {
    left_border = 0;
    right_border = 0;
    l_lost[row] = 1;
    r_lost[row] = 1;

    for (col = (int16_t)lwc_col; col >= 2; col--) {
      if (image_bin[row][col] == IMG_WHITE &&
          image_bin[row][col - 1] == IMG_BLACK &&
          image_bin[row][col - 2] == IMG_BLACK) {
        left_border = (int)col;
        l_lost[row] = 0;
        break;
      } else if (col <= 2) {
        left_border = (int)col;
        l_lost[row] = 1;
        break;
      }
    }

    for (col = (int16_t)lwc_col; col <= (int16_t)(IMG_W - 3); col++) {
      if (image_bin[row][col] == IMG_WHITE &&
          image_bin[row][col + 1] == IMG_BLACK &&
          image_bin[row][col + 2] == IMG_BLACK) {
        right_border = (int)col;
        r_lost[row] = 0;
        break;
      } else if (col >= (int16_t)(IMG_W - 1 - 2)) {
        right_border = (int)col;
        r_lost[row] = 1;
        break;
      }
    }

    l_border[row] = (uint8_t)left_border;
    r_border[row] = (uint8_t)right_border;
  }
  }

  outer_analyse();

  if (lwc_col <= 60u && left_lost_time >= 60u && right_lost_time <= 5u) {
    if (r_border[boundry_start_right] <= (IMG_W / 2)) {
      last_lwc_col = IMG_CENTER;
      lwc_col = IMG_CENTER;
      goto begin;
    }
  } else if (lwc_col >= 128u && right_lost_time >= 60u && left_lost_time <= 5u) {
    if (l_border[boundry_start_left] >= (IMG_W / 2)) {
      last_lwc_col = IMG_CENTER;
      lwc_col = IMG_CENTER;
      goto begin;
    }
  }
}

/* ================= 参考库 Outer_Analyse(去掉坡道/斑马线检测调用) ================= */

static void outer_analyse(void) {
  static uint8_t my_init_flag = 0;
  int16_t ir;

  left_lost_time = 0;
  right_lost_time = 0;
  both_lost_time = 0;
  boundry_start_left = 0;
  boundry_start_right = 0;

  for (ir = (int16_t)LWC_SCAN_START_ROW; ir >= 1; ir--) {
    if (l_lost[ir]) {
      left_lost_time++;
    }
    if (r_lost[ir]) {
      right_lost_time++;
    }
    if (l_lost[ir] && r_lost[ir]) {
      both_lost_time++;
    }
    if (boundry_start_left == 0 && !l_lost[ir]) {
      boundry_start_left = (uint8_t)ir;
    }
    if (boundry_start_right == 0 && !r_lost[ir]) {
      boundry_start_right = (uint8_t)ir;
    }
    road_wide[ir] = (uint8_t)(r_border[ir] - l_border[ir]);
  }

  if (my_init_flag == 0) {
    if (left_lost_time <= 15u && right_lost_time <= 15u && both_lost_time <= 15u) {
      road_type = LWC_ROAD_STRAIGHT;
    }
    if (left_lost_time < 15u && right_lost_time >= 30u && both_lost_time < 15u &&
        search_stop_line <= 100u) {
      road_type = LWC_ROAD_RIGHT_TURN;
    }
    if (right_lost_time < 15u && left_lost_time >= 30u && both_lost_time < 15u &&
        search_stop_line <= 100u) {
      road_type = LWC_ROAD_LEFT_TURN;
    }
    if (left_lost_time >= 15u && right_lost_time <= 5u && both_lost_time <= 5u &&
        search_stop_line >= 100u) {
      road_type = LWC_ROAD_LEFT_HUANDAO;
    }
    if (left_lost_time <= 5u && right_lost_time >= 15u && both_lost_time <= 5u &&
        search_stop_line >= 100u) {
      road_type = LWC_ROAD_RIGHT_HUANDAO;
    }
    if (right_lost_time >= 30u && left_lost_time >= 30u && both_lost_time >= 30u) {
      road_type = LWC_ROAD_CROSSING;
    }
    my_init_flag = 1;
  }

  if (road_type != LWC_ROAD_RAMP) {
    if (left_lost_time <= 15u && right_lost_time <= 15u && both_lost_time <= 15u) {
      road_type = LWC_ROAD_STRAIGHT;
    }
    if (left_lost_time < 15u && right_lost_time >= 30u && both_lost_time < 15u &&
        search_stop_line <= 100u) {
      road_type = LWC_ROAD_RIGHT_TURN;
    }
    if (right_lost_time < 15u && left_lost_time >= 30u && both_lost_time < 15u &&
        search_stop_line <= 100u) {
      road_type = LWC_ROAD_LEFT_TURN;
    }
    if (left_lost_time >= 15u && right_lost_time <= 5u && both_lost_time <= 5u &&
        search_stop_line >= 100u) {
      road_type = LWC_ROAD_LEFT_HUANDAO;
    }
    if (left_lost_time <= 5u && right_lost_time >= 15u && both_lost_time <= 5u &&
        search_stop_line >= 100u) {
      road_type = LWC_ROAD_RIGHT_HUANDAO;
    }
    if (right_lost_time >= 30u && left_lost_time >= 30u && both_lost_time >= 30u) {
      road_type = LWC_ROAD_CROSSING;
    }
  }
}

/* ================= 十字补线:参考库 Cross_Detect 链路 ================= */

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

static void add_line(uint8_t *border, uint8_t *lost, int x1, int y1, int x2,
                     int y2, track_info_t *ti) {
  int i, max, a1, a2, hx;

  if (x1 >= IMG_W)
    x1 = IMG_W - 1;
  else if (x1 <= 0)
    x1 = 0;
  if (x2 >= IMG_W)
    x2 = IMG_W - 1;
  else if (x2 <= 0)
    x2 = 0;
  if (y1 >= IMG_H)
    y1 = IMG_H - 1;
  else if (y1 <= 0)
    y1 = 0;
  if (y2 >= IMG_H)
    y2 = IMG_H - 1;
  else if (y2 <= 0)
    y2 = 0;

  if (y2 == y1) {
    return;
  }

  a1 = y1;
  a2 = y2;
  if (a1 > a2) {
    max = a1;
    a1 = a2;
    a2 = max;
  }

  for (i = a1; i <= a2; i++) {
    hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
    if (hx >= IMG_W)
      hx = IMG_W - 1;
    else if (hx <= 0)
      hx = 0;
    border[i] = (uint8_t)hx;
    lost[i] = 0;
  }
  mark_cross_fill_rows((uint8_t)a1, (uint8_t)a2, ti);
}

static void lengthen_boundry(uint8_t *border, uint8_t *lost, int start, int end,
                             track_info_t *ti) {
  int i, t;
  float k = 0.0f;

  if (start >= IMG_H - 1)
    start = IMG_H - 1;
  else if (start <= 0)
    start = 0;
  if (end >= IMG_H - 1)
    end = IMG_H - 1;
  else if (end <= 0)
    end = 0;
  if (end < start) {
    t = start;
    start = end;
    end = t;
  }

  if (start <= 5) {
    add_line(border, lost, border[start], start, border[end], end, ti);
  } else {
    k = (float)((int)border[start] - (int)border[start - 4]) / 5.0f;
    for (i = start; i <= end; i++) {
      int v = (int)((float)(i - start) * k) + (int)border[start];
      if (v >= IMG_W - 1)
        v = IMG_W - 1;
      else if (v <= 0)
        v = 0;
      border[i] = (uint8_t)v;
      lost[i] = 0;
    }
    mark_cross_fill_rows((uint8_t)start, (uint8_t)end, ti);
  }
}

static void find_down_point(int start, int end) {
  int i, t;

  down_find_l = 0;
  down_find_r = 0;

  if (start < end) {
    t = start;
    start = end;
    end = t;
  }
  if (start >= IMG_H - 1 - 5)
    start = IMG_H - 1 - 5;
  if (end <= (int)(IMG_H - search_stop_line))
    end = (int)(IMG_H - search_stop_line);
  if (end <= 5)
    end = 5;

  for (i = start; i >= end; i--) {
    if (down_find_l == 0 &&
        my_abs((int)l_border[i] - (int)l_border[i + 1]) <= 5 &&
        my_abs((int)l_border[i + 1] - (int)l_border[i + 2]) <= 5 &&
        my_abs((int)l_border[i + 2] - (int)l_border[i + 3]) <= 5 &&
        my_abs((int)l_border[i] - (int)l_border[i - 2]) >= 8 &&
        my_abs((int)l_border[i] - (int)l_border[i - 2]) >= 15 &&
        my_abs((int)l_border[i] - (int)l_border[i - 4]) >= 15) {
      down_find_l = i;
    }
    if (down_find_r == 0 &&
        my_abs((int)r_border[i] - (int)r_border[i + 1]) <= 5 &&
        my_abs((int)r_border[i + 1] - (int)r_border[i + 2]) <= 5 &&
        my_abs((int)r_border[i + 2] - (int)r_border[i + 3]) <= 5 &&
        my_abs((int)r_border[i] - (int)r_border[i - 2]) >= 8 &&
        my_abs((int)r_border[i] - (int)r_border[i - 2]) >= 15 &&
        my_abs((int)l_border[i] - (int)l_border[i - 4]) >= 15) {
      down_find_r = i;
    }
    if (down_find_l != 0 && down_find_r != 0) {
      break;
    }
  }
}

static void find_up_point(int start, int end) {
  int i, t;

  if (down_find_l != 0) {
    last_up_find_l = down_find_l;
  }
  if (down_find_r != 0) {
    last_up_find_r = down_find_r;
  }
  up_find_l = 0;
  up_find_r = 0;

  if (start < end) {
    t = start;
    start = end;
    end = t;
  }
  if (end <= (int)(IMG_H - search_stop_line))
    end = (int)(IMG_H - search_stop_line);
  if (end <= 5)
    end = 5;
  if (start >= IMG_H - 1 - 5)
    start = IMG_H - 1 - 5;

  for (i = end; i <= start; i++) {
    if (up_find_l == 0 &&
        my_abs((int)l_border[i] - (int)l_border[i - 1]) <= 5 &&
        my_abs((int)l_border[i - 1] - (int)l_border[i - 2]) <= 5 &&
        my_abs((int)l_border[i - 2] - (int)l_border[i - 3]) <= 5 &&
        my_abs((int)l_border[i] - (int)l_border[i + 2]) >= 8 &&
        my_abs((int)l_border[i] - (int)l_border[i + 3]) >= 15 &&
        my_abs((int)l_border[i] - (int)l_border[i + 4]) >= 15) {
      up_find_l = i;
    }
    if (up_find_r == 0 &&
        my_abs((int)r_border[i] - (int)r_border[i - 1]) <= 5 &&
        my_abs((int)r_border[i - 1] - (int)r_border[i - 2]) <= 5 &&
        my_abs((int)r_border[i - 2] - (int)r_border[i - 3]) <= 5 &&
        my_abs((int)r_border[i] - (int)r_border[i + 2]) >= 8 &&
        my_abs((int)r_border[i] - (int)r_border[i + 3]) >= 15 &&
        my_abs((int)r_border[i] - (int)r_border[i + 4]) >= 15) {
      up_find_r = i;
    }
    if (up_find_l != 0 && up_find_r != 0) {
      break;
    }
  }

  if (my_abs(up_find_r - up_find_l) >= 30 &&
      l_border[up_find_l] >= r_border[up_find_r]) {
    up_find_r = 0;
    up_find_l = 0;
  }
}

static void cross_detect(track_info_t *ti) {
  int down_search_start = 0;

  if (road_type != LWC_ROAD_CROSSING) {
    return;
  }

  up_find_l = 0;
  up_find_r = 0;

  if (both_lost_time >= 15u) {
    find_up_point(110, 6);
    if (up_find_l == 0 && up_find_r == 0) {
      return;
    }
  } else {
    return;
  }

  if (up_find_l != 0 && up_find_r != 0) {
    down_search_start =
        (up_find_l > up_find_r) ? up_find_l : up_find_r;
    find_down_point(IMG_H - 5, down_search_start + 10);
    if (down_find_l <= up_find_l) {
      down_find_l = 0;
    }
    if (down_find_r <= up_find_r) {
      down_find_r = 0;
    }

    if (down_find_l != 0 && down_find_r != 0) {
      add_line(l_border, l_lost, l_border[up_find_l], up_find_l,
               l_border[down_find_l], down_find_l, ti);
      add_line(r_border, r_lost, r_border[up_find_r], up_find_r,
               r_border[down_find_r], down_find_r, ti);
    } else if (down_find_l == 0 && down_find_r != 0) {
      lengthen_boundry(l_border, l_lost, up_find_l - 1, IMG_H - 1, ti);
      add_line(r_border, r_lost, r_border[up_find_r], up_find_r,
               r_border[down_find_r], down_find_r, ti);
    } else if (down_find_l != 0 && down_find_r == 0) {
      lengthen_boundry(r_border, r_lost, up_find_r - 1, IMG_H - 1, ti);
      add_line(l_border, l_lost, l_border[up_find_l], up_find_l,
               l_border[down_find_l], down_find_l, ti);
    } else {
      lengthen_boundry(l_border, l_lost, up_find_l - 1, IMG_H - 1, ti);
      lengthen_boundry(r_border, r_lost, up_find_r - 1, IMG_H - 1, ti);
    }

    ti->cross_valid = 1;
    ti->inflect_row = TR_ROW((uint8_t)up_find_l);
  }
}

static void export_track(track_info_t *ti) {
  int16_t ir;
  uint8_t tr;
  uint8_t both_lost = 0;
  int16_t lo;
  int16_t hi;

  for (tr = 0; tr < IMG_H; tr++) {
    ti->left[tr] = 0;
    ti->right[tr] = (uint8_t)(IMG_W - 1);
    ti->mid[tr] = IMG_CENTER;
    ti->left_lost[tr] = 1;
    ti->right_lost[tr] = 1;
  }

  if (stop_row > (int16_t)LWC_SCAN_START_ROW) {
    ti->both_lost_rows = 0;
    return;
  }

  lo = (int16_t)stop_row;
  hi = (int16_t)LWC_SCAN_START_ROW;

  for (ir = lo; ir <= hi; ir++) {
    tr = TR_ROW(ir);
    ti->left[tr] = clamp_u8(l_border[ir], 0, IMG_W - 1);
    ti->right[tr] = clamp_u8(r_border[ir], 0, IMG_W - 1);
    ti->mid[tr] =
        (uint8_t)(((uint16_t)ti->left[tr] + (uint16_t)ti->right[tr]) / 2u);
    ti->left_lost[tr] = l_lost[ir];
    ti->right_lost[tr] = r_lost[ir];
    if (ti->left_lost[tr] && ti->right_lost[tr]) {
      both_lost++;
    }
  }

  ti->both_lost_rows = both_lost;
}

static int16_t look_ahead_error(const track_info_t *ti, uint8_t *look_n_out) {
  const uint8_t span = (uint8_t)STEER_LOOK_SPAN;
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
    tr = (uint8_t)(TR_ROW(LWC_SCAN_START_ROW) + r);
    if (ti->left_lost[tr] && ti->right_lost[tr]) {
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
  return g_err_hold;
}

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out) {
  uint8_t th;
  uint8_t look_n;

  init_cross_meta(out);

  th = (image_threshold > 0) ? (uint8_t)image_threshold : otsu_threshold(img);
  out->threshold = th;

  binarize(img, th);
  image_filter(image_bin);
  image_draw_rectan(image_bin);

  center_line_deal_plus(LWC_SCAN_COL_MIN, LWC_SCAN_COL_MAX);
  if (image_cross_fill) {
    cross_detect(out);
  }
  export_track(out);

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
  uint8_t tr0 = (uint8_t)TR_ROW(LWC_SCAN_START_ROW);
  uint8_t any = 0;

  ips200_show_gray_image(0, 0, (const uint8 *)image_bin, IMG_W, IMG_H, IMG_W,
                         IMG_H, 128);

  if (lwc_len != 0u) {
    ips200_draw_line(lwc_col, (uint16_t)stop_row, lwc_col,
                     (uint16_t)LWC_SCAN_START_ROW, RGB565_PURPLE);
  }

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
