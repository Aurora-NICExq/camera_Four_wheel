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
    uint8_t  look_rows;
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
extern volatile uint16_t steer_look_far;

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out);
void image_debug_show(const track_info_t *ti);

#endif /* IMAGE_H */
