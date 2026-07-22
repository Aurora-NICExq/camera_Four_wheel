/* control.c - steer PD + duty cap */
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "control.h"

volatile float   steer_kp_min       = KP_MIN;
volatile float   steer_kp_max       = KP_MAX;
volatile float   steer_kp_e_sat     = KP_E_SAT;
volatile float   steer_kd           = KD;
volatile float   steer_d_filt_alpha = D_FILT_ALPHA;
volatile uint8_t  drive_armed       = 0;
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

static uint16_t lut_lookup_ascend(int32_t x, const int16_t *bins, const uint16_t *out, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        if (x < bins[i])
        {
            return out[i];
        }
    }
    return out[len - 1u];
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

    static const int16_t  rows_bins[ROWS_DUTY_TABLE_LEN] = ROWS_DUTY_TABLE_ROWS;
    static const uint16_t rows_duty[ROWS_DUTY_TABLE_LEN] = ROWS_DUTY_TABLE_DUTY;
    static const int16_t  curv_bins[CURV_DUTY_TABLE_LEN] = CURV_DUTY_TABLE_CURV;
    static const uint16_t curv_duty[CURV_DUTY_TABLE_LEN] = CURV_DUTY_TABLE_DUTY;

    uint16_t cap_rows = lut_lookup_ascend(ti->valid_rows, rows_bins, rows_duty, ROWS_DUTY_TABLE_LEN);

    int16_t curv_abs = (ti->curvature >= 0) ? ti->curvature : (int16_t)(-ti->curvature);
    uint16_t cap_curv = lut_lookup_ascend(curv_abs, curv_bins, curv_duty, CURV_DUTY_TABLE_LEN);

    int32_t steer_off = (int32_t)out->servo_pwm - SERVO_CENTER;
    if (steer_off < 0) steer_off = -steer_off;
    int32_t cap_steer_i = (int32_t)DUTY_HARD_CAP - (steer_off * STEER_DUTY_SLOPE_NUM) / STEER_DUTY_SLOPE_DEN;
    uint16_t cap_steer = (cap_steer_i < MIN_TURN_DUTY) ? MIN_TURN_DUTY : (uint16_t)cap_steer_i;

    uint16_t target = drive_duty_base;
    if (cap_rows  < target) target = cap_rows;
    if (cap_curv  < target) target = cap_curv;
    if (cap_steer < target) target = cap_steer;
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
