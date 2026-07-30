/* image.h - track_info_t + 最长白列巡线 */
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

    uint8_t  valid_rows;
    int16_t  error;
    /* error 的来源:0 = 本帧实测;1..ERR_HOLD_MAX_FRAMES = 盲区保持中,
       值为已保持的帧数,到上限后维持该值并每帧 ×3/4 衰减回中。
       没有这一列时,遥测里陈旧误差和实测误差长得完全一样,
       整段日志会被误当成实测数据来调 Kp/Kd */
    uint8_t  err_hold;
    /* 两段前瞻各自实际投票的行数(排除双边丢线行)。
       R6:归一化的分母必须可观测——far_rows=0 表示远段整段没信息,
       误差退化成纯近段,前瞻当帧失效。这是唯一的降级路径,必须看得见 */
    uint8_t  near_rows;
    uint8_t  far_rows;
    uint8_t  both_lost_rows;
    uint8_t  threshold;

    uint8_t  cross_filled[IMG_H];
    uint8_t  cross_valid;
} track_info_t;

extern volatile int16_t image_threshold;
extern volatile uint8_t image_cross_fill;
extern volatile uint16_t steer_far_w_pct; /* 菜单 Far W%:远段权重占比 0..100 */

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out);

/* Camera 调试:二值底图 + 彩色边线/中线叠加,纯显示不参与控制 */
void image_debug_show(const track_info_t *ti);

/* 摄像头校准:仅二值化、跳过 3x3 滤波,便于观察环岛等内部黑区 */
uint8_t image_calib_show(const uint8_t img[IMG_H][IMG_W]);
uint8_t image_calib_last_th(void);

uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe);

#endif /* IMAGE_H */
