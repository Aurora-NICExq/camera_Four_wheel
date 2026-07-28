/* image.h */
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
    uint8_t  err_held;   /* 本帧误差来自盲区保持而非实测,供控制侧抑制伪微分 */
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
extern volatile uint16_t steer_w_duty_ref;

void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out);

/* 调试显示:二值图叠加边线/中线,纯显示不参与控制 */
const uint8_t *image_debug_frame(const track_info_t *ti);

/* 复位帧间记忆状态(误差保持、占空比低通) */
void image_reset_state(void);

uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe);

#endif /* IMAGE_H */
