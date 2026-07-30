/* image.h - track_info_t + 八邻域双边巡线 */
#ifndef IMAGE_H
#define IMAGE_H

#include <stdint.h>
#include "config.h"

typedef struct
{
    uint8_t  left [IMG_H];
    uint8_t  right[IMG_H];
    uint8_t  mid  [IMG_H];
    uint8_t  width[IMG_H];
    uint8_t  left_lost [IMG_H];
    uint8_t  right_lost[IMG_H];

    uint8_t  valid_rows;
    int16_t  error;
    uint8_t  err_hold;
    uint8_t  look_rows; /* 本帧前瞻窗内实际参与行数;0 = 前瞻塌陷走 hold */
    uint8_t  both_lost_rows;
    uint8_t  threshold;

    uint8_t  cross_filled[IMG_H];
    uint8_t  cross_valid;
    uint8_t  cross_lo;
    uint8_t  cross_hi;
    uint8_t  inflect_row;
} track_info_t;

extern volatile int16_t image_threshold;
extern volatile uint8_t image_cross_fill;

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out);

/* Camera 调试:二值底图 + 彩色边线/中线叠加,纯显示不参与控制 */
void image_debug_show(const track_info_t *ti);

uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe);

#endif /* IMAGE_H */
