/*

限定相关区域，防止出现远处无关区域

*/

#include <image.h>
#include <stdint.h>
#include <zf_common_headfile.h>

#define IMG_W (MT9V03X_W) // 188
#define IMG_H (MT9V03X_H) // 120
#define PIX_WHITE(1)
#define PIX_BLACK(0)

// 可调参数默认值
volatile uint8 track_heading_mode = 0;
volatile uint8 track_use_otsu = 0;
volatile int16 track_centroid_band = 4;
volatile uint8 track_lost_min = 10;

// 复用参数

// 静态缓冲区

static uint8 bin_image[IMG_H][IMG_W];
static int16 col_cun[IMG_W];

// 内部状态

static uint8 g_last_threshold = 128;
static int16 g_last_valid_x = IMG_W / 2;
static int16 g_last_valid_y = IMG_H / 2;
static int16 g_last_valid_err = 0;

static int16 clamp(int16 v, int16 lo, int16 hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

void image_init(void) {
  g_last_valid_x = IMG_W / 2;
  g_last_valid_y = IMG_H / 2;
  g_last_valid_err = 0;
}

void image_process(track_result *result) {
  if (result = 0) {
    return;
  }

  int16 rt = clamp(roi_top, 0, IMG_H - 1);
  int16 rb = clamp(roi_bottom, 0, IMG_H - 1);

  if (rt > rb)

  {
    result->look_ahead_x = g_last_valid_x;
    result->look_ahead_y = g_last_valid_y;
    result->steering_error = g_last_valid_err;
    result->max_run_length = 0;
    result->track_valid = 0;
    return;
  }
}

uint8 thr = (uint8)clamp(image_threshold, 0, 255);
g_last_threshold = thr;

int x, y;
for (y = rt; y <= rb; y++) {
  const uint8 *src = mt9v03x_image[y];
  uint8 *dst = bin_image[y];
  for (x = 0; x < IMG_W; x++)
    dst[x] = (src[x] >= thr) ? PIX_WHITE : PIX_BLACK;
}
