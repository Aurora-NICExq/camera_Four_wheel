/* control.c - steer PD + 滑行式开环速度策略（FSM 契约） */
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "fsm.h"
#include "control.h"

volatile float   steer_kp_min       = KP_MIN;
volatile float   steer_kp_max       = KP_MAX;
volatile float   steer_kp_e_sat     = KP_E_SAT;
volatile uint8_t steer_use_const_kp = USE_CONST_KP;
volatile float   steer_kp_const     = KP_CONST;
volatile float   steer_kd           = KD;
volatile float   steer_d_filt_alpha = D_FILT_ALPHA;

static int16_t  g_prev_error;
static float    g_d_filt;
static uint16_t g_duty_now;
static uint8_t  g_exit_cnt;
static uint8_t  g_boost_cnt;
static uint8_t  g_boost_on;
static uint16_t g_servo_now;

uint16_t control_servo_clamp(int32_t servo_raw)
{
    if (servo_raw < SERVO_MIN) servo_raw = SERVO_MIN;
    if (servo_raw > SERVO_MAX) servo_raw = SERVO_MAX;
    return (uint16_t)servo_raw;
}

static int16_t steer_error_from_contract(const track_info_t *ti, const state_contract_t *ct)
{
    switch (ct->steer_src)
    {
    case STEER_SRC_LEFT_EDGE:
    case STEER_SRC_RIGHT_EDGE:
    {
        int32_t acc = 0;
        uint8_t cnt = 0;
        uint8_t r;
        uint8_t hi = (ti->valid_rows < CURV_NEAR_ROW_HI) ? ti->valid_rows : CURV_NEAR_ROW_HI;
        for (r = CURV_NEAR_ROW_LO; r < hi; r++)
        {
            int16_t mid;
            if (ct->steer_src == STEER_SRC_LEFT_EDGE)
            {
                mid = (int16_t)ti->left[r] + (int16_t)fsm_ring_half_width(r);
            }
            else
            {
                mid = (int16_t)ti->right[r] - (int16_t)fsm_ring_half_width(r);
            }
            acc += (mid - IMG_CENTER);
            cnt++;
        }
        return (cnt > 0) ? (int16_t)(acc / cnt) : 0;
    }
    case STEER_SRC_FIXED_BIAS:
        return ct->steer_bias;
    case STEER_SRC_MIDLINE:
    default:
        return ti->error;
    }
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
    g_exit_cnt   = 0;
    g_boost_cnt  = 0;
    g_boost_on   = 0;
    g_servo_now  = SERVO_CENTER;
}

void control_update(const track_info_t *ti, uint8_t armed, control_out_t *out)
{
    const state_contract_t *ct = fsm_contract();
    int16_t error = steer_error_from_contract(ti, ct);

    if (fsm_state_just_entered())
    {
        g_prev_error = error;
        g_d_filt     = 0.0f;
    }

    float d_raw = (float)(error - g_prev_error);
    g_prev_error = error;
    g_d_filt += steer_d_filt_alpha * (d_raw - g_d_filt);

    float kp;
    if (steer_use_const_kp)
    {
        kp = steer_kp_const;
    }
    else
    {
        float e_abs = (error >= 0) ? (float)error : (float)(-error);
        float ratio = e_abs / steer_kp_e_sat;
        if (ratio > 1.0f) ratio = 1.0f;
        kp = steer_kp_min + (steer_kp_max - steer_kp_min) * ratio * ratio;
    }

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

    int16_t err_abs = (ti->error >= 0) ? ti->error : (int16_t)(-ti->error);
    if (err_abs <= BOOST_ERR_MAX && ti->valid_rows >= BOOST_ROWS_MIN && curv_abs <= EXIT_CURV_MAX &&
        ti->both_lost_rows <= BOOST_MAX_BOTH_LOST &&
        fsm_state() == ST_NORMAL)
    {
        if (g_boost_cnt < 255u) g_boost_cnt++;
    }
    else
    {
        g_boost_cnt = 0;
    }
    if (curv_abs > BOOST_EXIT_CURV || fsm_state() != ST_NORMAL)
    {
        g_boost_on = 0;
    }
    else if (g_boost_cnt >= BOOST_CONFIRM_FRAMES)
    {
        g_boost_on = 1;
    }

    uint16_t target = g_boost_on ? BOOST_DUTY : STRAIGHT_DUTY;
    if (cap_rows  < target) target = cap_rows;
    if (cap_curv  < target) target = cap_curv;
    if (cap_steer < target) target = cap_steer;

    fsm_state_t speed_state = fsm_state();
    uint8_t severe_image = 0u;
    uint8_t ring_speed_override = 0u;
    if ((speed_state == ST_RING_PRE || speed_state == ST_RING_IN || speed_state == ST_RING_EXIT) &&
        ti->valid_rows >= ELEMENT_SPEED_MIN_ROWS && !image_track_invalid(ti, &severe_image))
    {
        target = ct->duty_cap;
        ring_speed_override = 1u;
    }

    if (err_abs <= EXIT_ERR_MAX && ti->valid_rows >= EXIT_ROWS_MIN && curv_abs <= EXIT_CURV_MAX &&
        ti->both_lost_rows <= EXIT_MAX_BOTH_LOST)
    {
        if (g_exit_cnt < 255u) g_exit_cnt++;
    }
    else
    {
        g_exit_cnt = 0;
    }
    if (!ring_speed_override && target > g_duty_now && g_exit_cnt < EXIT_CONFIRM_FRAMES)
    {
        uint16_t floor_duty = (g_duty_now > MIN_TURN_DUTY) ? g_duty_now : MIN_TURN_DUTY;
        if (target > floor_duty) target = floor_duty;
    }

    if (target > ct->duty_cap) target = ct->duty_cap;
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

    if (!armed)
    {
        g_duty_now  = 0;
        g_boost_on  = 0;
        g_boost_cnt = 0;
        g_exit_cnt  = 0;
    }

    out->duty       = g_duty_now;
    out->error_used = error;
}
