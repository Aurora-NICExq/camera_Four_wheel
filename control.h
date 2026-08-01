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
} control_out_t;

extern volatile float    steer_kp;
extern volatile float    steer_kd;
extern volatile float    steer_d_filt_alpha;
extern volatile uint8_t  drive_armed;
extern volatile uint8_t  drive_timed_out;
extern volatile uint16_t drive_stop_time_s;
extern volatile uint16_t drive_duty_base;
extern volatile uint16_t drive_failsafe_frames;

void control_init(void);
void control_duty_reset(void);
void control_update(const track_info_t *ti, control_out_t *out);
uint16_t control_servo_clamp(int32_t servo_raw);

#endif /* CONTROL_H */
