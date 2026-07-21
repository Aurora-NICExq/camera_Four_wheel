/* image.h - track_info_t + 八邻域 v2.0 巡线 detectors */
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
    uint8_t  longest_col;
    int16_t  error;
    int16_t  curvature;
    uint8_t  both_lost_rows;
    uint8_t  threshold;

    uint8_t  det_cross;
    uint8_t  det_ring_left;
    uint8_t  det_ring_right;
    uint8_t  det_ramp;
    uint8_t  inflect_row;       /* 十字补线起始行；无则 0xFF */
    int16_t  det_value;

    /* 十字补线（八邻域生长方向 + 最小二乘延伸） */
    uint8_t  cross_filled[IMG_H]; /* 1=该行坐标被十字连接覆盖 */
    uint8_t  cross_valid;         /* 本帧找到可靠十字连接 */
    uint8_t  cross_lo;            /* 补线区间起点（含） */
    uint8_t  cross_hi;            /* 补线区间终点（不含，左闭右开） */
} track_info_t;

/* 菜单可调二值化阈值：0 = 大津法；>0 = 固定阈值 */
extern volatile int16_t image_threshold;

void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out);
uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe);

uint8_t detect_cross     (const track_info_t *ti);
uint8_t detect_ring_left (const track_info_t *ti);
uint8_t detect_ring_right(const track_info_t *ti);
uint8_t detect_ramp      (const track_info_t *ti);

#endif /* IMAGE_H */
