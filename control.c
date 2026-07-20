/*********************************************************************************************************************
 * 模块：control.c — 转向 PD + 定速开环占空比（纯逻辑层，可在 PC 上用 gcc 编译）
 *
 * 精简版：已移除行数/曲率/转向减速表、boost、出弯再加速确认与环岛速度覆盖。
 * 速度策略简化为：target = min(STRAIGHT_DUTY, 契约 duty_cap, DUTY_HARD_CAP) + 升/降斜坡。
 ********************************************************************************************************************/
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "fsm.h"
#include "control.h"

/*===================================================================================================================
 * 一、内部状态（跨帧记忆）
 *==================================================================================================================*/

static int16_t  g_prev_error;
static float    g_d_filt;
static uint16_t g_duty_now;
static uint16_t g_servo_now;

/*===================================================================================================================
 * 二、内部工具
 *==================================================================================================================*/

uint16_t control_servo_clamp(int32_t servo_raw)
{
    if (servo_raw < (SERVO_CENTER - SERVO_RANGE))
    {
        servo_raw = SERVO_CENTER - SERVO_RANGE;
    }
    if (servo_raw > (SERVO_CENTER + SERVO_RANGE))
    {
        servo_raw = SERVO_CENTER + SERVO_RANGE;
    }
    return (uint16_t)servo_raw;
}

static int16_t steer_error_from_contract(const track_info_t *ti, const state_contract_t *ct)
{
    (void)ct;
    return ti->error;
}

/*===================================================================================================================
 * 三、对外接口
 *==================================================================================================================*/

void control_init(void)
{
    g_prev_error = 0;
    g_d_filt     = 0.0f;
    g_duty_now   = 0;
    g_servo_now  = SERVO_CENTER;
}

void control_update(const track_info_t *ti, uint8_t armed, control_out_t *out)
{
    const state_contract_t *ct = fsm_contract();

    /*==================================== 转向 PD ====================================*/
    int16_t error = steer_error_from_contract(ti, ct);

    if (fsm_state_just_entered())
    {
        g_prev_error = error;
        g_d_filt     = 0.0f;
    }

    float d_raw = (float)(error - g_prev_error);
    g_prev_error = error;
    g_d_filt += D_FILT_ALPHA * (d_raw - g_d_filt);

#if USE_CONST_KP
    float kp = KP_CONST;
#else
    float e_abs = (error >= 0) ? (float)error : (float)(-error);
    float ratio = e_abs / KP_E_SAT;
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    float kp = KP_MIN + (KP_MAX - KP_MIN) * ratio * ratio;
#endif

    int32_t servo_raw = SERVO_CENTER
                      + (int32_t)(SERVO_DIR * (kp * (float)error + KD * g_d_filt));

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

    /*==================================== 速度（定速开环） ====================================*/
    uint16_t target = STRAIGHT_DUTY;
    if (target > ct->duty_cap)
    {
        target = ct->duty_cap;
    }
    if (target > DUTY_HARD_CAP)
    {
        target = DUTY_HARD_CAP;
    }

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
        g_duty_now = 0;
    }

    out->duty       = g_duty_now;
    out->error_used = error;
}
