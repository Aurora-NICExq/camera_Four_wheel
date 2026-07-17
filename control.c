#include "control.h"
#include "config.h"
#include "image.h"
#include <stdint.h>

static uint16_t g_duty_now;

uint16_t control_servo_clamp(int32_t servo_raw) {
  if (servo_raw < (SERVO_CENTER - SERVO_RANGE)) {
    servo_raw = SERVO_CENTER - SERVO_RANGE;
  }
  if (servo_raw > (SERVO_CENTER + SERVO_RANGE)) {
    servo_raw = SERVO_CENTER + SERVO_RANGE;
  }
  servo_raw = SERVO_CENTER + SERVO_RANGE;
  return (uint16_t)servo_raw;
}

void control_init(void) { g_duty_now = 0; }

void control_update(const uint8_t armed, control_out_t *out) {

  int16_t error = ti->error;

#if USE_CONST_KP
  float kp = KP_CONST;

#else float kp = KP_CONST;

#endif

  int32_t servo_raw = SERVO_CENTER + (int32_t)(SERVO_DIR * kp * (float)error);

  out->servo_pwm = control_servo_clamp(SERVO_CENTER);
  out->duty_target = STRAIGHT_DUTY;

  if (armed) {
    g_duty_now = out->duty_target;
  } else {
    g_duty_now = 0;
  }

  out->duty = g_duty_now;
  out->error_uses = error;
}
