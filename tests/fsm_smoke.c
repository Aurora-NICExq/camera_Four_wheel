#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "image.h"
#include "fsm.h"

static void normal_frame(track_info_t *ti)
{
    memset(ti, 0, sizeof(*ti));
    ti->valid_rows = 100;
    memset(ti->width, WIDTH_TABLE_DEFAULT, sizeof(ti->width));
}

int main(void)
{
    track_info_t ti;
    int i;

    normal_frame(&ti);
    fsm_init();
    ti.det_cross = 1;
    ti.both_lost_rows = 20;
    fsm_update(&ti);
    ti.det_cross = 0;
    for (i = 0; i < CROSS_CONFIRM_N; i++)
    {
        fsm_update(&ti);
    }
    if (fsm_state() != ST_NORMAL)
    {
        return 1;
    }

    fsm_init();
    normal_frame(&ti);
    ti.det_cross = 1;
    ti.both_lost_rows = 20;
    for (i = 0; i < CROSS_CONFIRM_M; i++)
    {
        fsm_update(&ti);
    }
    if (fsm_state() != ST_CROSS)
    {
        return 2;
    }
    if (fsm_contract()->duty_cap != CROSS_DUTY_CAP)
    {
        return 3;
    }

    ti.det_cross = 0;
    ti.both_lost_rows = 20;
    for (i = 0; i <= CROSS_TIMEOUT_FRAMES; i++)
    {
        fsm_update(&ti);
    }
    if (fsm_state() != ST_RECOVERY)
    {
        return 4;
    }
    if (!fsm_trace(0) || fsm_trace(0)->trigger != -1)
    {
        return 5;
    }

    ti.valid_rows = 0;
    for (i = 0; i <= RECOVERY_TIMEOUT_FRAMES; i++)
    {
        fsm_update(&ti);
    }
    if (fsm_state() != ST_FAULT || !fsm_fault_request())
    {
        return 6;
    }

    puts("fsm smoke: PASS");
    return 0;
}
