#include "config.h"
#include "image.h"
#include "fsm.h"
#include "control.h"
#include "motor.h"

int main(void)
{
    track_info_t track = {0};
    control_out_t out = {0};
    fsm_state_t state = ST_NORMAL;
    state_contract_t contract = {STEER_SRC_MIDLINE, 0, 0, 0};

    (void)track;
    (void)out;
    (void)state;
    (void)contract;
    return 0;
}
