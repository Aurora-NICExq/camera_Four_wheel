/* control.h */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "config.h"
#include "image.h"

typedef struct
{
    uint16_t servo_pwm;
    uint16_t duty;
    int16_t  error_used;
    uint16_t duty_target;
    uint8_t  straight;   /* 本帧直道确认结果,供调试显示 */
    uint16_t rows_cap;   /* 本帧行数安全网上限,供调试显示:限速何时介入不再是隐形的 */
    int16_t  curv;       /* 本帧曲率读数(中线二阶差分定点值),供 Curv Div 标定 */
    uint16_t temp_pct;   /* 弯度 temp × 100,看减速实际吃了多少 */
} control_out_t;

extern volatile uint8_t  drive_armed;
extern volatile uint8_t  drive_timed_out;
extern volatile uint16_t drive_stop_time_s;
extern volatile uint16_t curve_temp_div;
extern volatile uint16_t drive_duty_base;
extern volatile uint16_t control_duty_prev;

void control_init(void);
void control_reset(void);
void control_duty_reset(void);
void control_update(const track_info_t *ti, control_out_t *out);
uint16_t control_servo_clamp(int32_t servo_raw);

#endif /* CONTROL_H */
