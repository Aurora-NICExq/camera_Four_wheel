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

/* 以下数组一律按图像行号 ir 索引(0=画面顶/最远,IMG_H-1=画面底/最近)。
   导出到 track_info_t 时才用 TR_ROW() 翻成"自底向上"的 tr。 */
static uint8_t l_border[IMG_H];
static uint8_t r_border[IMG_H];
static uint8_t l_lost[IMG_H];
static uint8_t r_lost[IMG_H];

static uint8_t white_col[IMG_W]; // 每列自底向上的连续白点数
static uint8_t lwc_len;          // 最长白列的长度
static uint8_t lwc_col;          // 最长白列的列号(扫边线的种子列)
static uint8_t stop_row;         // 有效行区间的上界(最小 ir);> LWC_START_ROW 表示无有效行

static int16_t g_err_hold;
static uint8_t g_hold_frames;

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

      if (num >= (uint32_t)LWC_FILTER_SUM_MAX && bin[r][c] == IMG_BLACK) {
        bin[r][c] = IMG_WHITE;
      }
      if (num <= (uint32_t)LWC_FILTER_SUM_MIN && bin[r][c] == IMG_WHITE) {
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

// 最长白列

/* 每列自 LWC_START_ROW 向上数连续白点,遇黑即停。
   底行就是黑的列直接得 0 —— 车正压在黑线上的那几列不会被选成种子。 */
static void count_white_columns(void) {
  uint16_t c;

  for (c = 0; c < IMG_W; c++) {
    white_col[c] = 0;
  }

  for (c = LWC_SCAN_COL_MIN; c <= (uint16_t)LWC_SCAN_COL_MAX; c++) {
    int16_t ir;
    uint8_t n = 0;
    for (ir = (int16_t)LWC_START_ROW; ir >= 0; ir--) {
      if (image_bin[ir][c] != IMG_WHITE) {
        break;
      }
      n++;
    }
    white_col[c] = n;
  }
}

/* 取最长的一列。并列时取最左的一列 —— 种子列只要落在赛道白带内,
   向左右扫出来的边线就是同一对,所以并列取哪一列不影响结果。 */
static void find_longest_white_column(void) {
  uint16_t c;

  lwc_len = 0;
  lwc_col = IMG_CENTER;

  for (c = LWC_SCAN_COL_MIN; c <= (uint16_t)LWC_SCAN_COL_MAX; c++) {
    if (white_col[c] > lwc_len) {
      lwc_len = white_col[c];
      lwc_col = (uint8_t)c;
    }
  }
}

/* 从种子列出发,每行向左/向右扫到"白黑黑"跳变。
   有效行区间 = 最长白列覆盖的那一段,即 [stop_row, LWC_START_ROW]。 */
static void scan_borders(void) {
  int16_t ir;

  for (ir = 0; ir < IMG_H; ir++) {
    l_border[ir] = LWC_BORDER_MIN;
    r_border[ir] = LWC_BORDER_MAX;
    l_lost[ir] = 1;
    r_lost[ir] = 1;
  }

  if (lwc_len == 0u) {
    stop_row = (uint8_t)(LWC_START_ROW + 1); // 无有效行
    return;
  }
  stop_row = (uint8_t)(LWC_START_ROW - (lwc_len - 1u));

  for (ir = (int16_t)LWC_START_ROW; ir >= (int16_t)stop_row; ir--) {
    int16_t j;

    for (j = (int16_t)lwc_col; j > (int16_t)LWC_BORDER_MIN; j--) {
      if (image_bin[ir][j] == IMG_WHITE && image_bin[ir][j - 1] == IMG_BLACK &&
          image_bin[ir][j - 2] == IMG_BLACK) {
        l_border[ir] = (uint8_t)j;
        l_lost[ir] = 0;
        break;
      }
    }

    for (j = (int16_t)lwc_col; j < (int16_t)LWC_BORDER_MAX; j++) {
      if (image_bin[ir][j] == IMG_WHITE && image_bin[ir][j + 1] == IMG_BLACK &&
          image_bin[ir][j + 2] == IMG_BLACK) {
        r_border[ir] = (uint8_t)j;
        r_lost[ir] = 0;
        break;
      }
    }
  }
}

// 十字补线

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

/* 写一个补出来的边线值。
   补过的行必须清丢线标志,否则 look_ahead_error 仍然跳过它们,补线等于白补。
   但补出画面外的行只钳位、不清丢线标志:那种行的真实边界在画面之外,
   钳位值是个假边界,标成"有效"等于凭空造了一个测量值。 */
static void put_border(uint8_t *border, uint8_t *lost, int16_t row, int32_t v) {
  border[row] = clamp_u8(v, LWC_BORDER_MIN, LWC_BORDER_MAX);
  lost[row] = (uint8_t)(v < LWC_BORDER_MIN || v > LWC_BORDER_MAX);
}

/* 两点连线补边:把 y1..y2 之间的边线换成 border[y1]→border[y2] 的直线。 */
static void fill_line(uint8_t *border, uint8_t *lost, uint8_t y1, uint8_t y2,
                      track_info_t *ti) {
  int16_t x1 = (int16_t)border[y1];
  int16_t x2 = (int16_t)border[y2];
  int16_t dy = (int16_t)y2 - (int16_t)y1;
  int16_t i;
  uint8_t a1 = (y1 < y2) ? y1 : y2;
  uint8_t a2 = (y1 < y2) ? y2 : y1;

  if (dy == 0) {
    return;
  }

  for (i = (int16_t)a1; i <= (int16_t)a2; i++) {
    put_border(border, lost, i,
               (int32_t)(i - (int16_t)y1) * (int32_t)(x2 - x1) / dy + x1);
  }
  mark_cross_fill_rows(a1, a2, ti);
}

/* 找不到下拐点时,拿拐点上方 SLOPE_SPAN 行的斜率把边线外推到画面底。
   基线取 8 行而不是参考实现的 4~5 行:斜率误差 ≈ ±1px/基线长度,
   外推 40 行后 4 行基线会放大到 ±10 列,8 行基线砍掉一半。
   (参考实现还有个 bug:跨 4 行却除以 5,斜率被压小 20%。) */
static void lengthen_boundry(uint8_t *border, uint8_t *lost, uint8_t start,
                             uint8_t end, track_info_t *ti) {
  int16_t k_num;
  int16_t i;

  if (start < (uint8_t)LWC_CROSS_SLOPE_SPAN || end <= start) {
    return;
  }

  k_num = (int16_t)border[start] -
          (int16_t)border[start - (uint8_t)LWC_CROSS_SLOPE_SPAN];

  {
    int32_t x0 = (int32_t)border[start];
    for (i = (int16_t)start; i <= (int16_t)end; i++) {
      put_border(border, lost, i,
                 x0 + (int32_t)(i - (int16_t)start) * (int32_t)k_num /
                          (int32_t)LWC_CROSS_SLOPE_SPAN);
    }
  }
  mark_cross_fill_rows(start, end, ti);
}

/* 上拐点:自上而下第一个"上方 3 行平滑、下方 3~4 行向外张开"的行。
   张开方向取带符号 —— 左边线必须向左跳、右边线必须向右跳,
   这样不用再加阈值就把"边线向内收"的噪声挡掉了。 */
static void find_up_points(uint8_t lo, uint8_t hi, uint8_t *up_l,
                           uint8_t *up_r) {
  int16_t i;

  *up_l = 0;
  *up_r = 0;

  for (i = (int16_t)lo; i <= (int16_t)hi; i++) {
    if (*up_l == 0 && my_abs((int)l_border[i] - (int)l_border[i - 1]) <=
                          LWC_CROSS_SMOOTH &&
        my_abs((int)l_border[i - 1] - (int)l_border[i - 2]) <=
            LWC_CROSS_SMOOTH &&
        my_abs((int)l_border[i - 2] - (int)l_border[i - 3]) <=
            LWC_CROSS_SMOOTH &&
        ((int)l_border[i] - (int)l_border[i + 3]) >= LWC_CROSS_JUMP &&
        ((int)l_border[i] - (int)l_border[i + 4]) >= LWC_CROSS_JUMP) {
      *up_l = (uint8_t)i;
    }
    if (*up_r == 0 && my_abs((int)r_border[i] - (int)r_border[i - 1]) <=
                          LWC_CROSS_SMOOTH &&
        my_abs((int)r_border[i - 1] - (int)r_border[i - 2]) <=
            LWC_CROSS_SMOOTH &&
        my_abs((int)r_border[i - 2] - (int)r_border[i - 3]) <=
            LWC_CROSS_SMOOTH &&
        ((int)r_border[i + 3] - (int)r_border[i]) >= LWC_CROSS_JUMP &&
        ((int)r_border[i + 4] - (int)r_border[i]) >= LWC_CROSS_JUMP) {
      *up_r = (uint8_t)i;
    }
    if (*up_l != 0 && *up_r != 0) {
      break;
    }
  }
}

/* 下拐点:自下而上第一个"下方 3 行平滑、上方 3~4 行向外张开"的行。 */
static void find_down_points(uint8_t lo, uint8_t hi, uint8_t *down_l,
                             uint8_t *down_r) {
  int16_t i;

  *down_l = 0;
  *down_r = 0;

  if (lo > hi) {
    return;
  }

  for (i = (int16_t)hi; i >= (int16_t)lo; i--) {
    if (*down_l == 0 && my_abs((int)l_border[i] - (int)l_border[i + 1]) <=
                            LWC_CROSS_SMOOTH &&
        my_abs((int)l_border[i + 1] - (int)l_border[i + 2]) <=
            LWC_CROSS_SMOOTH &&
        my_abs((int)l_border[i + 2] - (int)l_border[i + 3]) <=
            LWC_CROSS_SMOOTH &&
        ((int)l_border[i] - (int)l_border[i - 3]) >= LWC_CROSS_JUMP &&
        ((int)l_border[i] - (int)l_border[i - 4]) >= LWC_CROSS_JUMP) {
      *down_l = (uint8_t)i;
    }
    if (*down_r == 0 && my_abs((int)r_border[i] - (int)r_border[i + 1]) <=
                            LWC_CROSS_SMOOTH &&
        my_abs((int)r_border[i + 1] - (int)r_border[i + 2]) <=
            LWC_CROSS_SMOOTH &&
        my_abs((int)r_border[i + 2] - (int)r_border[i + 3]) <=
            LWC_CROSS_SMOOTH &&
        ((int)r_border[i - 3] - (int)r_border[i]) >= LWC_CROSS_JUMP &&
        ((int)r_border[i - 4] - (int)r_border[i]) >= LWC_CROSS_JUMP) {
      *down_r = (uint8_t)i;
    }
    if (*down_l != 0 && *down_r != 0) {
      break;
    }
  }
}

/* 开口区间内应当接近全宽。弯道也能凑出单侧拐点,但内侧边界仍在、张不开。
   区间 [lo,hi] 是**上拐点与下拐点之间的真实开口**,不是固定窗口:
   固定窗口在车离十字还远、开口只有几行时会一路采到开口外的正常赛道,
   宽行占比被稀释到 2/3 以下,恰好在最该补线的接近段把自己否掉。 */
static uint8_t cross_open_enough(uint8_t lo, uint8_t hi) {
  uint16_t row;
  uint8_t samples = 0;
  uint8_t open_cnt = 0;

  if (lo > hi) {
    return 0;
  }

  for (row = lo; row <= (uint16_t)hi; row++) {
    samples++;
    if (((int16_t)r_border[row] - (int16_t)l_border[row]) >=
        LWC_CROSS_OPEN_WIDTH) {
      open_cnt++;
    }
  }

  if (samples < (uint8_t)LWC_CROSS_OPEN_ROWS_MIN) {
    return 0; // 开口太短,不是十字
  }
  return (uint8_t)(((uint16_t)open_cnt * 3u) >= ((uint16_t)samples * 2u));
}

static void cross_fill(track_info_t *ti) {
  uint8_t lo;
  uint8_t hi;
  uint8_t up_l;
  uint8_t up_r;
  uint8_t down_l;
  uint8_t down_r;
  uint8_t base;
  uint8_t open_lo;
  uint8_t open_hi;
  uint8_t extrapolated = 0;

  if (stop_row > (uint8_t)LWC_START_ROW) {
    return;
  }

  /* 搜索带:上端要够 lengthen_boundry 取 SLOPE_SPAN 行基线(8 > EDGE_GUARD),
     下端要够拐点判据访问 i+EDGE_GUARD */
  lo = (uint8_t)(stop_row + LWC_CROSS_SLOPE_SPAN);
  hi = (uint8_t)(LWC_START_ROW - LWC_CROSS_EDGE_GUARD);
  if (lo > hi) {
    return;
  }

  /* 上拐点是判"是不是十字"的主判据:两侧必须同时出现。
     下拐点可有可无 —— 有就连线,没有就按斜率外推。 */
  find_up_points(lo, hi, &up_l, &up_r);
  if (up_l == 0 || up_r == 0) {
    return;
  }
  /* 真十字左右上拐点行号接近;弯道两侧拐点不同步 */
  if (my_abs((int)up_l - (int)up_r) > LWC_CROSS_BREAK_DROW) {
    return;
  }
  /* 左拐点必须在右拐点左边。两个拐点交叉说明它们根本不是同一个开口的两侧 */
  if (l_border[up_l] >= r_border[up_r]) {
    return;
  }

  base = (up_l > up_r) ? up_l : up_r;

  /* 下拐点只在上拐点下方找,且要给 i-EDGE_GUARD 留出余量。
     搜索下界 base+EDGE_GUARD 已经保证 down_* == 0 或 down_* > up_*,
     所以这里不需要再判一次"下拐点跑到上拐点上面去了" */
  find_down_points((uint8_t)(base + LWC_CROSS_EDGE_GUARD), hi, &down_l,
                   &down_r);

  /* 开口区间 = 上拐点下一行 → 最近的那个下拐点上一行;
     没有下拐点就说明开口一直延伸到画面最近处(车已在十字里) */
  open_lo = (uint8_t)(base + 1u);
  open_hi = (uint8_t)LWC_START_ROW;
  if (down_l != 0 && down_r != 0) {
    open_hi = (uint8_t)(((down_l < down_r) ? down_l : down_r) - 1u);
  } else if (down_l != 0) {
    open_hi = (uint8_t)(down_l - 1u);
  } else if (down_r != 0) {
    open_hi = (uint8_t)(down_r - 1u);
  }
  if (!cross_open_enough(open_lo, open_hi)) {
    return;
  }

  if (down_l != 0) {
    fill_line(l_border, l_lost, up_l, down_l, ti);
  } else {
    lengthen_boundry(l_border, l_lost, up_l, (uint8_t)LWC_START_ROW, ti);
    extrapolated = 1;
  }

  if (down_r != 0) {
    fill_line(r_border, r_lost, up_r, down_r, ti);
  } else {
    lengthen_boundry(r_border, r_lost, up_r, (uint8_t)LWC_START_ROW, ti);
    extrapolated = 1;
  }

  /* 1 = 上下拐点都在、两侧都是连线(接近段,几何最可靠)
     2 = 至少一侧靠斜率外推(车已进十字,近端赛道已不可见)
     调试页 CRS 直接看得到走的是哪条路径 */
  ti->cross_valid = (uint8_t)(extrapolated ? 2u : 1u);
  ti->inflect_row = TR_ROW(up_l);
}

static void export_track(track_info_t *ti) {
  int16_t ir;
  uint8_t tr;
  uint8_t both_lost = 0;

  for (tr = 0; tr < IMG_H; tr++) {
    ti->left[tr] = 0;
    ti->right[tr] = (uint8_t)(IMG_W - 1);
    ti->mid[tr] = IMG_CENTER;
    ti->left_lost[tr] = 1;
    ti->right_lost[tr] = 1;
  }

  if (stop_row > (uint8_t)LWC_START_ROW) {
    ti->both_lost_rows = 0;
    return;
  }

  for (ir = (int16_t)stop_row; ir <= (int16_t)LWC_START_ROW; ir++) {
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
    tr = (uint8_t)(TR_ROW(LWC_START_ROW) + r);
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

  th = (image_threshold > 0) ? (uint8_t)image_threshold : otsu_threshold(img);
  out->threshold = th;

  binarize(img, th);
  image_filter(image_bin);
  image_draw_rectan(image_bin);

  count_white_columns();
  find_longest_white_column();
  scan_borders();
  if (image_cross_fill) {
    cross_fill(out);
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
  uint8_t tr0 = (uint8_t)TR_ROW(LWC_START_ROW);
  uint8_t any = 0;

  ips200_show_gray_image(0, 0, (const uint8 *)image_bin, IMG_W, IMG_H, IMG_W,
                         IMG_H, 128);

  /* 最长白列本身画出来:紫色竖线的列号 = 扫边线的种子列,
     线的上端 = stop_row = 本帧有效视野的顶。这是新机制唯一的内部状态。
     注意屏幕 y 直接等于图像行号 ir(下面边线那圈 y=IMG_H-1-tr,而
     tr=IMG_H-1-ir,两次翻转抵消),所以这里**不能**再套一次 TR_ROW。 */
  if (lwc_len != 0u) {
    ips200_draw_line(lwc_col, (uint16_t)stop_row, lwc_col,
                     (uint16_t)LWC_START_ROW, RGB565_PURPLE);
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
