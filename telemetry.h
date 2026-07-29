/* telemetry.h - 无线串口 CSV 遥测,供 PC/MATLAB 调参分析 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "image.h"
#include "control.h"

extern volatile uint8_t telemetry_enable;

void telemetry_init(void);
void telemetry_pump(void);
void telemetry_update(uint32_t t_ms, uint32_t frame,
                      const track_info_t *ti, const control_out_t *out);

#endif /* TELEMETRY_H */
