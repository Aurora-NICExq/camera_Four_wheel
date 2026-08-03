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
    uint8_t  left_lost [IMG_H];
    uint8_t  right_lost[IMG_H];

    int16_t  error;
    uint8_t  err_hold;
    uint8_t  aim_row;        /* 本帧瞄准行(tr);0 = 无有效行,丢线保护据此判定 */
    uint8_t  both_lost_rows;
    uint8_t  threshold;

    /* 直道/弯道判别(见 config.h CURVE_VAR_* 说明)。未接入控制,仅上屏。 */
    uint16_t mid_var;        /* Σ(mid-IMG_CENTER)²/n,判据本体 */
    uint16_t mid_var_ac;     /* 同一批样本对自身均值的方差,诊断用:横偏 vs 弯曲 */
    uint8_t  mid_var_rows;   /* 实际参与统计的行数;不足 CURVE_VAR_MIN_ROWS 时上面两个数无效 */
    uint8_t  is_curve;       /* mid_var >= curve_var_th;行数不足时恒为 0 */

    uint8_t  cross_filled[IMG_H];
    uint8_t  cross_valid;
    uint8_t  cross_lo;
    uint8_t  cross_hi;
    uint8_t  inflect_row;
} track_info_t;

extern volatile int16_t image_threshold;
extern volatile uint8_t image_cross_fill;
extern volatile uint16_t steer_look_far;
extern volatile uint16_t curve_var_th;

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out);
void image_debug_show(const track_info_t *ti);

#endif /* IMAGE_H */
