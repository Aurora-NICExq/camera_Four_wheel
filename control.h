#ifndef MOTOL_CONTROL_H
#define MOTOL_CONTROL_H

#include "config.h"
#include "fsm.h"
#include "image.h"
#include <cstdint>
#include <stdint.h>

typedef struct {
  uint16_t servo_pwm;
  uint16_t duty;
  int16_t error_uses;
  uint16_t duty_target;
} control_out_t;

void control_init(void);

void control_update(const track_info_t *ti, uint8_t armed, control_out_t *out);

uint16_t control_servo_clamp(int32_t servo_raw);

#endif
