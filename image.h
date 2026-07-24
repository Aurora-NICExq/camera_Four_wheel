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
    uint8_t  both_lost_rows;
    uint8_t  threshold;
} track_info_t;

extern volatile int16_t image_threshold;

void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out);

uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe);

#endif /* IMAGE_H */
