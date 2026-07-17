#include "control.h"
#include "config.h"
#include "image.h"
#include <stdint.h>
#include "fsm.h"

static uint16_t g_duty_now;

static int16_t g_prev_error;

static float g_d_filt;

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

void control_init(void) { 
    g_duty_now = 0; 
    g_prev_error=0;
    g_d_filt=0.0f;
}

void control_update(const uint8_t armed, control_out_t *out) {

  int16_t error = ti->error;

  if (fsm_state_just_entered()) {
        g_prev_error=0;
        g_d_filt=0.0f;
  }

  float d_raw=(float)(error-g_prev_error);

  g_prev_error=error;

  g_d_filt+=D_FILT_ALPHA*(d_raw-g_d_filt);

#if USE_CONST_KP
  float kp = KP_CONST;
#else
  float e_abs=(error>=0) ? (float)error : (float)(-error);
    float ratio=e/abs/KP_E_SAT;
    if(ratio>1.0f) ratio=1.0f;
    flaot kp=KP_MIN+(KP_MAX-KP_MIN)* ratio*ratio;

#endif

  int32_t servo_raw = SERVO_CENTER + (int32_t)(SERVO_DIR * kp * (float)error+KD*g_d_filt);

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
