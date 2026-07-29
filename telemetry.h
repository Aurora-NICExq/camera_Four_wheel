#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "image.h"
#include "control.h"

extern volatile uint8_t telemetry_enable;

void telemetry_init(void);
void telemetry_reinit(void);
void telemetry_pump(void);
void telemetry_update(uint32_t t_ms, uint32_t frame,
                      const track_info_t *ti, const control_out_t *out);

uint8_t  telemetry_test_send(uint32_t seq, uint32_t t_ms);
uint8_t  telemetry_test_banner(void);
uint8_t  telemetry_wireless_ok(void);
uint32_t telemetry_tx_bytes(void);
uint32_t telemetry_baud(void);
uint16_t telemetry_queue_depth(void);
uint8_t  telemetry_rts_blocked(void);

#endif
