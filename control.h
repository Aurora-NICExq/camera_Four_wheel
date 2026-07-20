/* control.h - steer PD + open-loop duty */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "config.h"
#include "image.h"
#include "fsm.h"

typedef struct
{
    uint16_t servo_pwm;
    uint16_t duty;
    int16_t  error_used;
    uint16_t duty_target;
} control_out_t;

void control_init(void);
void control_update(const track_info_t *ti, uint8_t armed, control_out_t *out);
uint16_t control_servo_clamp(int32_t servo_raw);

#endif /* CONTROL_H */
