/* display.h - telemetry + buzzer API */
#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "image.h"
#include "control.h"
#include "fsm.h"

void display_init(void);
void display_update(void);
void display_chirp_fault(void);
void display_chirp(fsm_state_t state);
void display_telemetry(const track_info_t *ti, const control_out_t *out, uint32_t frame);

#endif /* DISPLAY_H */
