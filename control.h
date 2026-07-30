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
    uint16_t duty_target; /* slew 之前的目标 = 菜单 Duty,用于区分"还在爬升"与"已到目标" */
} control_out_t;

/* control.c 的整定量。此前只在 menu.c / menu_config.c 里各自 extern,
   telemetry.c 直接引用 steer_kp/kd 却没有任何声明在作用域内(隐式声明,
   非法 C)。统一收到定义所在模块的头文件里 */
extern volatile float    steer_kp;
extern volatile float    steer_kd;
extern volatile float    steer_d_filt_alpha;

extern volatile uint8_t  drive_armed;
extern volatile uint8_t  drive_timed_out;
extern volatile uint16_t drive_stop_time_s;
extern volatile uint16_t drive_duty_base;

void control_init(void);
void control_reset(void);
void control_duty_reset(void);
void control_update(const track_info_t *ti, control_out_t *out);
uint16_t control_servo_clamp(int32_t servo_raw);

#endif /* CONTROL_H */
