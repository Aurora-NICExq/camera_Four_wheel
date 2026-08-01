# image.c 逐行注释

> ## ⚠️ 已过期 — 本文描述的是**八邻域**版 `image.c`
>
> `feat/longest-white-column` 分支已把巡线整体换成**最长白列 + 十字补线**:
> 删除 `get_start_point` / `search_l_r` / `get_left` / `get_right` /
> `points_l·r` / `dir_l·r`,以及基于方向序列 + 最小二乘的 `cross_fill`;
> 新增 `count_white_columns` / `find_longest_white_column` / `scan_borders`,
> `cross_fill` 改为上下拐点 + 连线/外推。`config.h` 的 `EIGHTN_*` 更名为 `LWC_*`。
>
> **下文的行号、函数名、常量名全部作废**,只有以下几处仍然有效:
> Otsu 阈值、`binarize`、`image_filter`、`image_draw_rectan`、
> `TR_ROW()` 坐标映射、`look_ahead_error`、`image_debug_show` 的绘图逻辑。
>
> 提交前需按新 `image.c` 重新生成本文。

---


> 行号与源文件一致。

---

```c
#include "image.h"                                // L1:  track_info_t 与对外接口声明
#include "config.h"                               // L2:  图像尺寸、八邻域、前瞻、十字补线常量
#include "zf_common_headfile.h"                   // L3:  逐飞库（ips200 调试绘图等）
#include <stdint.h>                               // L4:  标准整数类型

volatile int16_t image_threshold = 0;             // L6:  菜单可调二值化阈值；0=每帧自动 Otsu
volatile uint8_t image_cross_fill = 1;            // L7:  十字补线开关；1=启用 cross_fill
volatile uint16_t steer_look_far = STEER_LOOK_FAR_DEFAULT; // L8: 前瞻起点（track 行号方向，菜单 Look Far）

#define IMG_WHITE (0xFFu)                         // L10: 二值图白色像素值 255
#define IMG_BLACK (0x00u)                         // L11: 二值图黑色像素值 0
#define TR_ROW(ir) ((uint8_t)(IMG_H - 1u - (uint8_t)(ir)))
                                                  // L12: 图像行号 ir → track 行号 tr
                                                  //      图像底部 ir=119 → tr=0（近车）
                                                  //      图像顶部 ir=0   → tr=119（远）

static uint8_t image_bin[IMG_H][IMG_W];           // L14: 二值化结果缓冲 [图像行][列]
static uint8_t l_border[IMG_H];                   // L15: 每图像行的左边界列号
static uint8_t r_border[IMG_H];                   // L16: 每图像行的右边界列号

static uint16_t points_l[EIGHTN_MAX_POINTS][2];   // L18: 左蚂蚁轨迹点 [x,y]，最多 360 个
static uint16_t points_r[EIGHTN_MAX_POINTS][2];   // L19: 右蚂蚁轨迹点
static uint16_t dir_l[EIGHTN_MAX_POINTS];         // L20: 左蚂蚁每步方向编号 0~7
static uint16_t dir_r[EIGHTN_MAX_POINTS];         // L21: 右蚂蚁每步方向编号

static uint8_t start_point_l[2];                  // L23: 左起点 [x, y]
static uint8_t start_point_r[2];                  // L24: 右起点 [x, y]
static uint16_t data_stastics_l;                  // L25: 左边记录的点数（statistics 拼写变体）
static uint16_t data_stastics_r;                  // L26: 右边记录的点数
static uint8_t hightest_row;                      // L27: 左右相遇时的图像行号（highest 拼写变体）
static uint8_t cross_break_l;                     // L28: 左十字拐点图像行号（补线用）
static uint8_t cross_break_r;                     // L29: 右十字拐点图像行号
static uint8_t cross_flag;                        // L30: 本帧是否执行了十字补线（内部标志）
static int16_t g_err_hold;                        // L31: 前瞻丢线时保持/衰减的误差值
static uint8_t g_hold_frames;                     // L32: 连续全丢前瞻行计数（用于衰减）

// ─────────────── 工具函数 ───────────────

static int my_abs(int v) { return (v >= 0) ? v : -v; }
                                                  // L34: 求绝对值

static int16_t limit_a_b(int16_t x, int16_t a, int16_t b) {
                                                  // L36: 有符号 16 位限幅 [a, b]
  if (x < a)
    return a;                                     // L37: 低于下限返回 a
  if (x > b)
    return b;                                     // L39: 高于上限返回 b
  return x;                                       // L41: 在范围内原样返回
}

static void init_cross_meta(track_info_t *ti) {   // L44: 每帧重置十字补线元数据
  uint8_t r;
  ti->cross_valid = 0;                            // L46: 本帧未确认十字
  ti->cross_lo = 0;                               // L47: 补线区 track 下界（近端）清零
  ti->cross_hi = 0;                               // L48: 补线区 track 上界（远端）清零
  ti->inflect_row = 0xFF;                         // L49: 拐点 track 行；0xFF=无效
  for (r = 0; r < IMG_H; r++) {
    ti->cross_filled[r] = 0;                      // L51: 每 track 行补线标记清零
  }
}

static uint8_t clamp_u8(int32_t v, int32_t lo, int32_t hi) {
                                                  // L55: 无符号 8 位限幅
  if (v < lo)
    return (uint8_t)lo;                           // L57: 低于下限
  if (v > hi)
    return (uint8_t)hi;                           // L58: 高于上限
  return (uint8_t)v;                              // L60: 范围内转 uint8_t
}

// ─────────────── Otsu 自动阈值 ───────────────

// 大津法                                          // L63: 段注释

static uint8_t otsu_threshold(const uint8_t img[IMG_H][IMG_W]) {
                                                  // L65: 从灰度图计算最佳二值化阈值
  uint32_t hist[256] = {0};                       // L66: 灰度直方图
  uint32_t total = 0;                             // L67: 参与统计的总像素数
  uint16_t r, c, i;                               // L68: 循环变量

  // 间隔采样                                      // L70: 降采样以省 CPU

  for (r = 0; r < IMG_H; r += OTSU_ROW_STEP) {    // L72: 隔 2 行
    for (c = 0; c < IMG_W; c += OTSU_COL_STEP) {  // L73: 隔 2 列
      hist[img[r][c]]++;                          // L74: 该灰度计数 +1
      total++;                                    // L75: 总像素 +1
    }
  }

  uint64_t sum_all = 0;                           // L79: 所有像素灰度值之和
  for (i = 0; i < 256; i++) {
    sum_all += (uint64_t)i * hist[i];             // L81: 累加 灰度×个数
  }

  uint64_t best_var = 0;                          // L84: 目前最大类间方差
  uint16_t best_th = FIXED_THRESHOLD;             // L85: 最佳阈值初值 128
  uint32_t w0 = 0;                                // L86: 暗部（≤阈值）像素累计数
  uint64_t sum0 = 0;                              // L87: 暗部灰度和

  for (i = 0; i < 256; i++) {                     // L89: 枚举候选阈值 i
    w0 += hist[i];                                // L90: 把灰度 i 归入暗部
    if (w0 == 0)
      continue;                                   // L91: 暗部为空，跳过
    uint32_t w1 = total - w0;                     // L93: 亮部像素数
    if (w1 == 0)
      break;                                      // L94: 亮部为空，结束枚举
    sum0 += (uint64_t)i * hist[i];                // L95: 暗部灰度和累加

    int64_t diff = (int64_t)(sum0 * w1) - (int64_t)((sum_all - sum0) * w0);
                                                  // L98: 两组均值差之分子
    uint64_t d2 = (uint64_t)((diff < 0) ? -diff : diff);
                                                  // L99: 取绝对值
    uint64_t var = (d2 / w0) * (d2 / w1);         // L100: 类间方差近似
    if (var > best_var) {                         // L101: 更大方差
      best_var = var;                             // L102
      best_th = i;                                // L103: 更新最佳阈值
    }
  }

  if (best_th < OTSU_THRESHOLD_MIN)               // L107: 下限钳位 40
    best_th = OTSU_THRESHOLD_MIN;
  if (best_th > OTSU_THRESHOLD_MAX)               // L109: 上限钳位 200
    best_th = OTSU_THRESHOLD_MAX;
  return (uint8_t)best_th;                        // L111: 返回最终阈值
}

// ─────────────── 二值化 ───────────────

// 二值化处理                                      // L114: 段注释

static void binarize(const uint8_t img[IMG_H][IMG_W], uint8_t th) {
                                                  // L116: 灰度图 → 二值图 image_bin
  uint16_t r, c;
  for (r = 0; r < IMG_H; r++) {
    for (c = 0; c < IMG_W; c++) {
      image_bin[r][c] = (img[r][c] >= th) ? IMG_WHITE : IMG_BLACK;
                                                  // L120: ≥th→白，<th→黑
    }
  }
}

// ─────────────── 八邻域滤波 ───────────────

static void image_filter(uint8_t bin[IMG_H][IMG_W]) {
                                                  // L125: 孤立点去噪
  uint16_t r, c;
  for (r = 1; r < IMG_H - 1; r++) {               // L127: 跳过最上/最下行
    for (c = 1; c < IMG_W - 1; c++) {             // L128: 跳过最左/最右列
      uint32_t num = bin[r - 1][c - 1] + bin[r - 1][c] + bin[r - 1][c + 1] +
                     bin[r][c - 1] + bin[r][c + 1] + bin[r + 1][c - 1] +
                     bin[r + 1][c] + bin[r + 1][c + 1];
                                                  // L129-131: 8 邻域灰度和（白=255）
      if (num >= (uint32_t)EIGHTN_FILTER_SUM_MAX && bin[r][c] == IMG_BLACK) {
                                                  // L133: 周围≥5 白、中心黑→填白
        bin[r][c] = IMG_WHITE;
      }
      if (num <= (uint32_t)EIGHTN_FILTER_SUM_MIN && bin[r][c] == IMG_WHITE) {
                                                  // L136: 周围≤2 白、中心白→填黑
        bin[r][c] = IMG_BLACK;
      }
    }
  }
}

// ─────────────── 画黑框（边界保护）───────────────

// 降噪保护                                        // L143: 段注释

static void image_draw_rectan(uint8_t bin[IMG_H][IMG_W]) {
                                                  // L145: 涂黑图像四边，防蚂蚁爬出界
  uint16_t r, c;
  for (r = 0; r < IMG_H; r++) {
    bin[r][0] = IMG_BLACK;                        // L148: 最左列
    bin[r][1] = IMG_BLACK;                        // L149: 左第 2 列
    bin[r][IMG_W - 1] = IMG_BLACK;                // L150: 最右列
    bin[r][IMG_W - 2] = IMG_BLACK;                // L151: 右第 2 列
  }
  for (c = 0; c < IMG_W; c++) {
    bin[0][c] = IMG_BLACK;                        // L154: 顶行
    bin[1][c] = IMG_BLACK;                        // L155: 第 2 行
  }                                               // 底边不涂，搜索从底部行开始
}

// ─────────────── 找左右起点 ───────────────

// 起点                                            // L159: 段注释

static uint8_t get_start_point(uint8_t start_row) {
                                                  // L161: 在 start_row 上找左右赛道边缘起点
  uint16_t i;
  uint8_t l_found = 0;
  uint8_t r_found = 0;

  start_point_l[0] = 0;                           // L166-169: 初始化起点
  start_point_l[1] = 0;
  start_point_r[0] = 0;
  start_point_r[1] = 0;

  // 从中线开始左右找                              // L171: 从图像中心列向两侧扫

  for (i = IMG_W / 2; i > EIGHTN_BORDER_MIN; i--) {
                                                  // L173: 列 94 → 1
    start_point_l[0] = (uint8_t)i;
    start_point_l[1] = start_row;
    if (image_bin[start_row][i] == IMG_WHITE &&
        image_bin[start_row][i - 1] == IMG_BLACK) {
                                                  // L176-177: 白|黑过渡 = 左边界内侧
      l_found = 1;
      break;
    }
  }

  for (i = IMG_W / 2; i < EIGHTN_BORDER_MAX; i++) {
                                                  // L183: 列 94 → 186
    start_point_r[0] = (uint8_t)i;
    start_point_r[1] = start_row;
    if (image_bin[start_row][i] == IMG_WHITE &&
        image_bin[start_row][i + 1] == IMG_BLACK) {
                                                  // L186-187: 白|黑过渡 = 右边界内侧
      r_found = 1;
      break;
    }
  }

  return (uint8_t)(l_found && r_found);           // L193: 两边都找到才返回 1
}

// ─────────────── 八邻域双边搜索（核心）───────────────

static void search_l_r(uint16_t break_flag, uint8_t bin[IMG_H][IMG_W],
                       uint16_t *l_stastic, uint16_t *r_stastic,
                       uint8_t l_start_x, uint8_t l_start_y, uint8_t r_start_x,
                       uint8_t r_start_y, uint8_t *hightest) {
                                                  // L196: 左右蚂蚁沿边界向上爬
  static const int8_t seeds_l[8][2] = {{0, 1},  {-1, 1}, {-1, 0}, {-1, -1},
                                       {0, -1}, {1, -1}, {1, 0},  {1, 1}};
                                                  // L200-201: 左蚂蚁 8 方向 [dx,dy]
                                                  // 0下 1左下 2左 3左上 4上 5右上 6右 7右下
  static const int8_t seeds_r[8][2] = {{0, 1},  {1, 1},   {1, 0},  {1, -1},
                                       {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
                                                  // L202-203: 右蚂蚁 8 方向（镜像顺序）

  uint8_t search_filds_l[8][2];                   // L205: 左 8 邻居坐标
  uint8_t search_filds_r[8][2];                   // L206: 右 8 邻居坐标
  uint8_t temp_l[8][2];                           // L207: 左候选下一中心
  uint8_t temp_r[8][2];                           // L208: 右候选下一中心
  uint8_t center_point_l[2];                      // L209: 左蚂蚁当前 [x,y]
  uint8_t center_point_r[2];                      // L210: 右蚂蚁当前 [x,y]
  uint8_t index_l;                                // L211: 左候选个数
  uint8_t index_r;                                // L212: 右候选个数
  uint8_t i, j;                                   // L213-214: 循环变量
  uint16_t l_data_statics = *l_stastic;           // L215: 左已记录点数
  uint16_t r_data_statics = *r_stastic;           // L216: 右已记录点数

  center_point_l[0] = l_start_x;                  // L218-221: 移到起点
  center_point_l[1] = l_start_y;
  center_point_r[0] = r_start_x;
  center_point_r[1] = r_start_y;

  while (break_flag--) {                          // L223: 最多 EIGHTN_MAX_POINTS 次
    for (i = 0; i < 8; i++) {                     // L224: 左 8 邻居
      search_filds_l[i][0] = (uint8_t)(center_point_l[0] + seeds_l[i][0]);
      search_filds_l[i][1] = (uint8_t)(center_point_l[1] + seeds_l[i][1]);
    }
    points_l[l_data_statics][0] = center_point_l[0];  // L228-229: 记录左中心
    points_l[l_data_statics][1] = center_point_l[1];
    l_data_statics++;                             // L230: 左点数 +1

    for (i = 0; i < 8; i++) {                     // L232: 右 8 邻居（本步先不推进）
      search_filds_r[i][0] = (uint8_t)(center_point_r[0] + seeds_r[i][0]);
      search_filds_r[i][1] = (uint8_t)(center_point_r[1] + seeds_r[i][1]);
    }
    points_r[r_data_statics][0] = center_point_r[0];  // L236-237: 记录右中心
    points_r[r_data_statics][1] = center_point_r[1];

    index_l = 0;                                  // L239: 左候选清零
    for (i = 0; i < 8; i++) {
      temp_l[i][0] = 0;
      temp_l[i][1] = 0;
    }

    for (i = 0; i < 8; i++) {                     // L245: 按方向序找左「黑|白」边界
      if (bin[search_filds_l[i][1]][search_filds_l[i][0]] == IMG_BLACK &&
          bin[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] ==
              IMG_WHITE) {
        temp_l[index_l][0] = search_filds_l[i][0];
        temp_l[index_l][1] = search_filds_l[i][1];
        dir_l[l_data_statics - 1] = i;            // L251: 记录本步方向
        index_l++;
      }

      if (index_l) {                              // L255: 有候选时选 y 最小（最靠上）
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

    // ── 停止条件 A：爬不动 ──
    if ((r_data_statics >= 2u &&                  // L267: 右连续 3 点相同
         points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] &&
         points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] &&
         points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] &&
         points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) ||
        (l_data_statics >= 3u &&                  // L272: 或左连续 3 点相同
         points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
         points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
         points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
         points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1])) {
      break;
    }

    // ── 停止条件 B：左右相遇 ──
    if (my_abs((int)points_r[r_data_statics][0] -
               (int)points_l[l_data_statics - 1][0]) < EIGHTN_MEET_DIST &&
        my_abs((int)points_r[r_data_statics][1] -
               (int)points_l[l_data_statics - 1][1]) < EIGHTN_MEET_DIST) {
                                                  // L280-283: x、y 差均 <2
      *hightest = (uint8_t)((points_r[r_data_statics][1] +
                             points_l[l_data_statics - 1][1]) >>
                            1);
                                                  // L284-286: 相遇行 = 两点 y 均值
      break;
    }

    // ── 同步：右比左更靠上则本轮不推进右 ──
    if (points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]) {
                                                  // L290: 右 y 更小=更靠图像上方
      continue;
    }

    // ── 左回退：方向 7 且右更低时撤销左最后一步 ──
    if (dir_l[l_data_statics - 1] == 7 &&        // L294: 左最后是右下
        points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]) {
                                                  // L295: 右比左更靠下（更近车）
      center_point_l[0] = (uint8_t)points_l[l_data_statics - 1][0];
      center_point_l[1] = (uint8_t)points_l[l_data_statics - 1][1];
      l_data_statics--;                           // L298: 撤销左最后记录
    }
    r_data_statics++;                             // L300: 右正式记录本步

    // ── 右边一步（逻辑同左）──
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

  *l_stastic = l_data_statics;                    // L331: 写回左总点数
  *r_stastic = r_data_statics;                    // L332: 写回右总点数
}

// ─────────────── 轨迹点 → 每行边界 ───────────────

static void get_left(uint16_t total_l) {          // L335: 左轨迹 → l_border[]
  uint16_t j;
  int16_t h = (int16_t)EIGHTN_START_ROW;          // L337: 从行 118 往上填
  uint8_t i;

  for (i = 0; i < IMG_H; i++) {
    l_border[i] = EIGHTN_BORDER_MIN;              // L341: 默认 1 = 左丢线
  }

  for (j = 0; j < total_l; j++) {
    if (points_l[j][1] == (uint16_t)h) {          // L345: 匹配当前图像行
      l_border[h] = (uint8_t)(points_l[j][0] + 1u);
                                                  // L346: 边界取轨迹点右侧一列（白侧）
    } else {
      continue;
    }
    h--;                                          // L350: 往上一行
    if (h <= 0) {
      break;                                      // L352: 到顶停止
    }
  }
}

static void get_right(uint16_t total_r) {         // L357: 右轨迹 → r_border[]（对称）
  uint16_t j;
  int16_t h = (int16_t)EIGHTN_START_ROW;
  uint8_t i;

  for (i = 0; i < IMG_H; i++) {
    r_border[i] = EIGHTN_BORDER_MAX;              // L363: 默认 186 = 右丢线
  }

  for (j = 0; j < total_r; j++) {
    if (points_r[j][1] == (uint16_t)h) {
      r_border[h] = (uint8_t)(points_r[j][0] - 1u);
                                                  // L368: 边界取轨迹点左侧一列
    } else {
      continue;
    }
    h--;
    if (h <= 0) {
      break;
    }
  }
}

// ─────────────── 十字补线 ───────────────

// 十字补线                                        // L379: 段注释

static float slope_calculate(uint8_t begin, uint8_t end,
                             const uint8_t *border) {
                                                  // L381: 最小二乘拟合斜率
  float xsum = 0.0f;
  float ysum = 0.0f;
  float xysum = 0.0f;
  float x2sum = 0.0f;
  int16_t i;
  float result = 0.0f;
  static float result_last;                       // L389: 分母为 0 时沿用上次斜率

  for (i = (int16_t)begin; i < (int16_t)end; i++) {
    xsum += (float)i;                             // L392: 行号
    ysum += (float)border[i];                     // L393: 边界列号
    xysum += (float)i * (float)border[i];
    x2sum += (float)i * (float)i;
  }

  if (((float)(end - begin) * x2sum - xsum * xsum) != 0.0f) {
    result = (((float)(end - begin) * xysum - xsum * ysum) /
              ((float)(end - begin) * x2sum - xsum * xsum));
                                                  // L399-400: 斜率公式
    result_last = result;
  } else {
    result = result_last;                         // L403: 退化时用上次值
  }
  return result;
}

static void calculate_s_i(uint8_t start, uint8_t end, uint8_t *border,
                          float *slope_rate, float *intercept) {
                                                  // L408: 拟合直线 y = kx + b
  uint16_t i;
  uint16_t num = 0;
  uint16_t xsum = 0;
  uint16_t ysum = 0;
  float x_average;
  float y_average;

  for (i = start; i < end; i++) {                 // L417: 统计窗口内均值
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

  // 直线拟合                                      // L431: 段注释

  *slope_rate = slope_calculate(start, end, border);
                                                  // L433: 斜率 k
  *intercept = y_average - (*slope_rate) * x_average;
                                                  // L434: 截距 b = ȳ - k·x̄
}

static void mark_cross_fill_rows(uint8_t y_lo, uint8_t y_hi, track_info_t *ti) {
                                                  // L437: 标记被补线的 track 行
  uint8_t a1 = y_lo;
  uint8_t a2 = y_hi;
  uint8_t i;

  if (a1 > a2) {                                  // L442: 保证 a1≤a2
    uint8_t t = a1;
    a1 = a2;
    a2 = t;
  }

  for (i = a1; i <= a2; i++) {
    uint8_t tr = TR_ROW(i);                       // L449: 图像行 → track 行
    if (tr < IMG_H) {
      ti->cross_filled[tr] = 1;                   // L451: 该行边界被补线改写
    }
  }

  if (ti->cross_lo == 0 && ti->cross_hi == 0) {   // L455: 首次标记时设置补线区范围
    ti->cross_lo = TR_ROW(a2);                    // L456: 近端（图像下方）track 行
    ti->cross_hi = (uint8_t)(TR_ROW(a1) + 1u);    // L457: 远端上界（开区间习惯 +1）
  }
}

static void cross_fill(uint8_t bin[IMG_H][IMG_W], track_info_t *ti) {
                                                  // L461: 检测十字并用直线外推补边界
  uint16_t i;
  uint8_t start;
  uint8_t end;
  float slope_rate = 0.0f;
  float intercept = 0.0f;
  uint8_t fill_from;

  cross_break_l = 0;                              // L469-471: 重置本帧十字状态
  cross_break_r = 0;
  cross_flag = 0;

  // ── 方向序列检测：4,4,6,6,6 ──
  for (i = 1; i + 7u < data_stastics_l; i++) {
    if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 &&
        dir_l[i + 5] == 6 && dir_l[i + 7] == 6) {
                                                  // L474-475: 左拐点特征序列
                                                  // 4=上 6=右：入十字前上爬、出十字右转
      cross_break_l = (uint8_t)points_l[i][1];    // L476: 记录拐点图像行
      break;
    }
  }

  for (i = 1; i + 7u < data_stastics_r; i++) {
    if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 &&
        dir_r[i + 5] == 6 && dir_r[i + 7] == 6) {
                                                  // L482-483: 右拐点对称序列（右蚂蚁方向镜像）
      cross_break_r = (uint8_t)points_r[i][1];
      break;
    }
  }

  if (!cross_break_l || !cross_break_r) {         // L489: 任一侧未检测到拐点
    return;
  }

  if (!bin[IMG_H - 1][EIGHTN_CROSS_CORNER_L] ||   // L493-495: 底行两角须为白
      !bin[IMG_H - 1][EIGHTN_CROSS_CORNER_R]) {   //      十字赛道底边连通性检查
    return;
  }

  /* 拟合窗口需完整落在图像内 */                   // L498-501: break-SLOPE_BACK 防下溢
  if (cross_break_l <= EIGHTN_CROSS_SLOPE_BACK ||
      cross_break_r <= EIGHTN_CROSS_SLOPE_BACK) {
    return;
  }

  /* 真十字:左右上拐点行号接近,且拐点下方开口接近全宽。
     弯道拐点同样能凑出 4,4,6,6,6 方向序列,但内侧边界仍在、宽度不足,
     误补线会把弯道中线拉直、掏空转向误差 */       // L504-506: 误检防护说明
  if (my_abs((int)cross_break_l - (int)cross_break_r) > EIGHTN_CROSS_BREAK_DROW) {
                                                  // L507: 左右拐点行差 >15 → 非十字
    return;
  }
  {
    uint8_t base =
        (cross_break_l > cross_break_r) ? cross_break_l : cross_break_r;
                                                  // L511-512: 取较高（更靠下）拐点为基准
    uint16_t row;
    uint8_t samples = 0;
    uint8_t open_cnt = 0;
    for (row = (uint16_t)base + 2u;
         row < (uint16_t)base + 12u && row <= (uint16_t)EIGHTN_CROSS_OPEN_ROW_MAX;
         row++) {                                 // L516-518: 拐点下方最多 10 行采样
      samples++;
      if (((int16_t)r_border[row] - (int16_t)l_border[row]) >=
          EIGHTN_CROSS_OPEN_WIDTH) {
                                                  // L520-521: 开口宽度 ≥140 像素
        open_cnt++;
      }
    }
    if (samples >= 4u && ((uint16_t)open_cnt * 3u) < ((uint16_t)samples * 2u)) {
                                                  // L525: 不足 2/3 行达标 → 弯道，放弃补线
      return;
    }
  }

  cross_flag = 1;                                 // L530: 确认执行补线
  fill_from = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_NEAR);
                                                  // L531: 左补线起始行 = 拐点上 5 行

  // ── 左边界外推 ──
  start = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_BACK);
                                                  // L533: 拟合窗起点 = 拐点上 15 行
  start = (uint8_t)limit_a_b((int16_t)start, 0, IMG_H - 1);
  end = (uint8_t)(cross_break_l - EIGHTN_CROSS_SLOPE_NEAR);
                                                  // L535: 拟合窗终点 = 拐点上 5 行
  calculate_s_i(start, end, l_border, &slope_rate, &intercept);
  for (i = fill_from; i < (uint16_t)(IMG_H - 1); i++) {
    int16_t v = (int16_t)(slope_rate * (float)i + intercept);
                                                  // L538: 直线预测列号
    l_border[i] = (uint8_t)limit_a_b(v, EIGHTN_BORDER_MIN, EIGHTN_BORDER_MAX);
    mark_cross_fill_rows((uint8_t)i, (uint8_t)i, ti);
                                                  // L540: 标记该行已补线
  }

  // ── 右边界外推（对称）──
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

  ti->cross_valid = 1;                            // L554: 本帧十字补线有效
  ti->inflect_row = TR_ROW(cross_break_l);        // L555: 左拐点 track 行号（遥测/调试）
}

// ─────────────── 导出 track_info_t ───────────────

static void export_track(track_info_t *ti, uint8_t hightest) {
                                                  // L558: l_border/r_border → track 数组
  uint8_t ir;                                     // L559: 图像行号
  uint8_t tr;                                     // L560: track 行号
  uint8_t both_lost = 0;                          // L561: 双丢行计数
  uint8_t lo = hightest;                          // L562: 有效区上界（图像行，小=远）
  uint8_t hi = EIGHTN_START_ROW;                  // L563: 有效区下界 = 118（近）

  for (tr = 0; tr < IMG_H; tr++) {                // L565: 初始化全部为丢线默认
    ti->left[tr] = 0;
    ti->right[tr] = (uint8_t)(IMG_W - 1);
    ti->mid[tr] = IMG_CENTER;
    ti->left_lost[tr] = 1;
    ti->right_lost[tr] = 1;
  }

  if (lo > hi) {                                  // L573: 上界>下界=搜索无效
    ti->both_lost_rows = 0;
    return;
  }

  for (ir = lo; ir <= hi; ir++) {                 // L578: 遍历有效图像行
    tr = TR_ROW(ir);                              // L579: 翻转为 track 行
    ti->left[tr] = clamp_u8(l_border[ir], 0, IMG_W - 1);
    ti->right[tr] = clamp_u8(r_border[ir], 0, IMG_W - 1);
    ti->mid[tr] =
        (uint8_t)(((uint16_t)ti->left[tr] + (uint16_t)ti->right[tr]) / 2u);
                                                  // L582-583: 中线 = 左右均值
    ti->left_lost[tr] = (uint8_t)(l_border[ir] <= (EIGHTN_BORDER_MIN +
                                                   EIGHTN_EDGE_LOST_MARGIN));
                                                  // L584-585: 左边界贴近左边缘 → 丢线
    ti->right_lost[tr] = (uint8_t)(r_border[ir] >= (EIGHTN_BORDER_MAX -
                                                    EIGHTN_EDGE_LOST_MARGIN));
                                                  // L586-587: 右边界贴近右边缘 → 丢线
    if (ti->left_lost[tr] && ti->right_lost[tr]) {
      both_lost++;                                // L589: 双丢计数
    }
  }

  ti->both_lost_rows = both_lost;                 // L593: 写入双丢行总数
}

// ─────────────── 单段前瞻误差 ───────────────

// 前瞻代码使用，超念是对的                          // L596: 段注释（作者备忘）

static int16_t look_ahead_error(const track_info_t *ti, uint8_t *look_n_out) {
                                                  // L598: 从 Look Far 起点向上取 span 行平均偏差
  const uint8_t span = (uint8_t)STEER_LOOK_SPAN;  // L599: 固定取 20 行
  int32_t acc = 0;                                // L600: 偏差累加
  uint8_t n = 0;                                  // L601: 实际参与行数
  uint8_t r;
  uint16_t far = steer_look_far;                  // L603: 菜单前瞻起点

  if (far > (uint16_t)STEER_LOOK_FAR_MAX) {       // L605-607: 上限 119
    far = (uint16_t)STEER_LOOK_FAR_MAX;
  }
  if (far <= (uint16_t)span) {                    // L608-610: 保证起点高于 span
    far = (uint16_t)span + 1u;
  }
  r = (uint8_t)far;                               // L611: 从远到近扫描的游标

  while (r > 0u && n < span) {                    // L613: 最多取 span 行
    uint8_t tr;

    r--;                                          // L616: 向近端（track 行号减小）移动
    tr = (uint8_t)(TR_ROW(EIGHTN_START_ROW) + r);
                                                  // L617: 换算为 track 行号
    if (ti->left_lost[tr] && ti->right_lost[tr]) {
                                                  // L618: 双丢行跳过，不计入平均
      continue;
    }
    acc += (int16_t)ti->mid[tr] - IMG_CENTER;     // L621: 累加 mid - 中心
    n++;
  }

  *look_n_out = n;                                // L625: 输出实际参与行数
  if (n == 0u) {                                  // L626: 前瞻窗口内全丢
    if (g_hold_frames < ERR_HOLD_MAX_FRAMES) {    // L627-629: 前 20 帧保持上次误差
      g_hold_frames++;
    } else {
      g_err_hold = (int16_t)((g_err_hold * 3) / 4);
                                                  // L630: 之后每帧衰减 25%
    }
    return g_err_hold;
  }
  g_hold_frames = 0;                              // L634: 有有效行则清零保持计数
  g_err_hold = (int16_t)(acc / (int32_t)n);       // L635: 更新并保持误差
  return g_err_hold;                              // L636: 平均偏差 = 转向 error
}

// ─────────────── 总入口 image_process ───────────────

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out) {
                                                  // L639: 每帧调用；不再收 duty 入参
  uint8_t th;
  uint8_t look_n;

  init_cross_meta(out);                           // L643: 重置十字元数据
  hightest_row = 0;                               // L644: 重置相遇行
  data_stastics_l = 0;                            // L645: 左点数清零
  data_stastics_r = 0;                            // L646: 右点数清零

  th = (image_threshold > 0) ? (uint8_t)image_threshold : otsu_threshold(img);
                                                  // L648: 手动阈值优先，否则 Otsu
  out->threshold = th;                            // L649: 记录本帧阈值

  binarize(img, th);                              // L651: 灰度→二值
  image_filter(image_bin);                        // L652: 八邻域去噪
  image_draw_rectan(image_bin);                   // L653: 边界涂黑

  if (get_start_point(EIGHTN_START_ROW)) {        // L655: 底行找到左右起点
    search_l_r((uint16_t)EIGHTN_MAX_POINTS, image_bin, &data_stastics_l,
               &data_stastics_r, start_point_l[0], start_point_l[1],
               start_point_r[0], start_point_r[1], &hightest_row);
                                                  // L656-658: 八邻域双边搜索
    get_left(data_stastics_l);                    // L659: 左边界整理
    get_right(data_stastics_r);                   // L660: 右边界整理
    if (image_cross_fill) {                       // L661: 菜单开关
      cross_fill(image_bin, out);                 // L662: 十字补线
    }
    export_track(out, hightest_row);              // L664: 导出 track
  } else {
    export_track(out, EIGHTN_START_ROW + 1u);     // L666: lo=119>hi=118 → 全丢线默认
  }

  out->error = look_ahead_error(out, &look_n);    // L669: 计算转向误差
  out->look_rows = look_n;                        // L670: 前瞻实际行数（遥测）
  out->err_hold = g_hold_frames;                  // L671: 丢线保持帧数（遥测）
}

// ─────────────── 调试显示 image_debug_show ───────────────

static void debug_draw_seg(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                           uint16_t color) {
                                                  // L674: 退化线段画点，否则画线
  if (x0 == x1 && y0 == y1) {
    ips200_draw_point(x0, y0, color);
  } else {
    ips200_draw_line(x0, y0, x1, y1, color);
  }
}

void image_debug_show(const track_info_t *ti) {   // L683: IPS200 叠加显示边界
  uint8_t tr;
  uint8_t tr0 = (uint8_t)TR_ROW(EIGHTN_START_ROW);
                                                  // L685: 搜索起点对应 track 行
  uint8_t any = 0;                                // L686: 是否画过任意线段

  ips200_show_gray_image(0, 0, (const uint8 *)image_bin, IMG_W, IMG_H, IMG_W,
                         IMG_H, 128);
                                                  // L688-689: 底图显示二值图

  for (tr = (uint8_t)(tr0 + 1u); tr < IMG_H; tr++) {
                                                  // L691: 从起点上一行往远端画
    uint8_t prev = (uint8_t)(tr - 1u);
    uint16_t y0 = (uint16_t)(IMG_H - 1u - prev);  // L693: track 行 → 屏幕 y
    uint16_t y1 = (uint16_t)(IMG_H - 1u - tr);
    uint8_t filled0 = ti->cross_filled[prev];     // L695-696: 补线行标记
    uint8_t filled1 = ti->cross_filled[tr];

    if (!ti->left_lost[prev] && !ti->left_lost[tr]) {
      debug_draw_seg(ti->left[prev], y0, ti->left[tr], y1,
                     (filled0 || filled1) ? RGB565_YELLOW : RGB565_BLUE);
                                                  // L698-700: 左边界；补线行黄色
      any = 1;
    }
    if (!ti->right_lost[prev] && !ti->right_lost[tr]) {
      debug_draw_seg(ti->right[prev], y0, ti->right[tr], y1,
                     (filled0 || filled1) ? RGB565_YELLOW : RGB565_RED);
                                                  // L703-705: 右边界；补线行黄色
      any = 1;
    }
    if (!ti->left_lost[prev] && !ti->right_lost[prev] && !ti->left_lost[tr] &&
        !ti->right_lost[tr]) {
      debug_draw_seg(ti->mid[prev], y0, ti->mid[tr], y1, RGB565_GREEN);
                                                  // L708-710: 中线始终绿色
      any = 1;
    }
  }

  if (!any && (!ti->left_lost[tr0] || !ti->right_lost[tr0])) {
                                                  // L715: 只有起点一行有效时画点
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
```

---

*行号与 `image.c` 一一对应（当前 729 行）。若源码增删行，请以 git 版本为准核对。*
