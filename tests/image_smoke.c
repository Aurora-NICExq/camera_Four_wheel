#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "image.h"

static uint8_t frame[IMG_H][IMG_W];

static void make_straight(void)
{
    int raw;
    int col;

    for (raw = 0; raw < IMG_H; raw++)
    {
        int near = IMG_H - 1 - raw;
        int half = 75 - (55 * near) / (IMG_H - 1);

        for (col = 0; col < IMG_W; col++)
        {
            frame[raw][col] =
                (col >= IMG_CENTER - half && col <= IMG_CENTER + half) ? 200 : 30;
        }
    }
}

int main(void)
{
    track_info_t ti;
    uint8_t severe;

    make_straight();
    image_process(frame, 0, &ti);
    if (ti.valid_rows < 80 || ti.error < -2 || ti.error > 2)
    {
        return 1;
    }
    if (image_track_invalid(&ti, &severe))
    {
        return 2;
    }

    memset(frame, 0, sizeof(frame));
    image_process(frame, 0, &ti);
    if (!image_track_invalid(&ti, &severe) || !severe)
    {
        return 3;
    }

    memset(frame, 255, sizeof(frame));
    image_process(frame, 0, &ti);
    if (!image_track_invalid(&ti, &severe) || !severe)
    {
        return 4;
    }

    puts("image smoke: PASS");
    return 0;
}
