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

/* 菜单 UART Test 页用:与 CSV 遥测共用同一条队列与 RTS 流控,
   所以自检通过就等于遥测链路通。返回 1 = 已入队,0 = 队列满被丢 */
uint8_t  telemetry_test_send(uint32_t seq, uint32_t t_ms);
uint8_t  telemetry_test_banner(void);
uint16_t telemetry_queue_depth(void);
uint8_t  telemetry_rts_blocked(void);

#endif /* TELEMETRY_H */
