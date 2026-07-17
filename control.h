#ifndef CONTROL_H
#define CONTROL_H

#include "config.h"
#include "fsm.h"
#include "image.h"
#include <cstdint>
#include <stdint.h>

#define SERVO_CENTER (750)
#define SERVO_RANGE (750)
#define STRAIGHT_DUTY (750)

#define SERVO_DIR (+1)
#define USE_CONST_KP (1)
#define KP_CONST (1.8f)

#define KD (6.0f)
#define D_FILT_ALPHA (0.04f)

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
