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
} control_out_t;

extern volatile float    steer_kp;
extern volatile float    steer_kp2;
extern volatile float    steer_ki;
extern volatile float    steer_kd;
extern volatile float    steer_ka;
extern volatile float    steer_up;
extern volatile float    speed_up;
extern volatile int16_t  straight_judge;
extern volatile int16_t  straight_judge_13;
extern volatile uint8_t  drive_armed;
extern volatile uint16_t drive_duty_base;
extern volatile uint16_t control_duty_prev;

void control_init(void);
void control_reset(void);
void control_duty_reset(void);
void control_update(const track_info_t *ti, control_out_t *out);
uint16_t control_servo_clamp(int32_t servo_raw);

#endif /* CONTROL_H */
