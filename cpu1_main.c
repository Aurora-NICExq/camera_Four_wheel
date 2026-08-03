/* cpu1_main.c */

#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"

void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);

    cpu_wait_event_ready();
    while (TRUE)
    {

    }
}
#pragma section all restore
