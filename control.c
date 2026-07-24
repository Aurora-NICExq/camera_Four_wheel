/* control.c - steer PD + steer-based duty cap */
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "control.h"

volatile float   steer_kp_min       = KP_MIN;
volatile float   steer_kp_max       = KP_MAX;
volatile float   steer_kp_e_sat     = KP_E_SAT;
volatile float   steer_kd           = KD;
volatile float   steer_d_filt_alpha = D_FILT_ALPHA;
volatile uint16_t drive_duty_base   = STRAIGHT_DUTY;
volatile uint16_t control_duty_prev = 0;

static int16_t  g_prev_error;
static float    g_d_filt;
static uint16_t g_duty_now;
static uint16_t g_servo_now;

uint16_t control_servo_clamp(int32_t servo_raw)
{
    if (servo_raw < SERVO_MIN) servo_raw = SERVO_MIN;
    if (servo_raw > SERVO_MAX) servo_raw = SERVO_MAX;
    return (uint16_t)servo_raw;
}

void control_init(void)
{
    g_prev_error = 0;
    g_d_filt     = 0.0f;
    g_duty_now   = 0;
    g_servo_now  = SERVO_CENTER;
}

void control_reset(void)
{
    control_init();
    control_duty_prev = 0;
}

void control_duty_reset(void)
{
    g_duty_now = 0;
}

void control_update(const track_info_t *ti, control_out_t *out)
{
    int16_t error = ti->error;

    float d_raw = (float)(error - g_prev_error);
    g_prev_error = error;
    g_d_filt += steer_d_filt_alpha * (d_raw - g_d_filt);

    float e_abs = (error >= 0) ? (float)error : (float)(-error);
    float ratio = e_abs / steer_kp_e_sat;
    if (ratio > 1.0f) ratio = 1.0f;
    float kp = steer_kp_min + (steer_kp_max - steer_kp_min) * ratio * ratio;

    int32_t servo_raw = SERVO_CENTER
                      + (int32_t)(SERVO_DIR * (kp * (float)error + steer_kd * g_d_filt));

    uint16_t servo_target = control_servo_clamp(servo_raw);
    if (servo_target > g_servo_now)
    {
        uint16_t step = (uint16_t)(servo_target - g_servo_now);
        g_servo_now += (step > SERVO_SLEW_LIMIT) ? SERVO_SLEW_LIMIT : step;
    }
    else
    {
        uint16_t step = (uint16_t)(g_servo_now - servo_target);
        g_servo_now -= (step > SERVO_SLEW_LIMIT) ? SERVO_SLEW_LIMIT : step;
    }
    out->servo_pwm = control_servo_clamp(g_servo_now);

    int32_t steer_off = (int32_t)out->servo_pwm - SERVO_CENTER;
    if (steer_off < 0) steer_off = -steer_off;

    uint16_t target = drive_duty_base;
    if (steer_off >= STEER_TURN_DUTY_PWM && target > STRAIGHT_DUTY)
    {
        target = STRAIGHT_DUTY;
    }
    if (target > DUTY_HARD_CAP) target = DUTY_HARD_CAP;

    out->duty_target = target;

    if (target > g_duty_now)
    {
        uint16_t step = (uint16_t)(target - g_duty_now);
        g_duty_now += (step > DUTY_SLEW_UP) ? DUTY_SLEW_UP : step;
    }
    else
    {
        uint16_t step = (uint16_t)(g_duty_now - target);
        g_duty_now -= (step > DUTY_SLEW_DOWN) ? DUTY_SLEW_DOWN : step;
    }

    out->duty       = g_duty_now;
    out->error_used = error;
}
