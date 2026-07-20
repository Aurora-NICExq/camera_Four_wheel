/*********************************************************************************************************************
 * 模块：control.c — 转向 PD + 定速开环占空比（纯逻辑层，可在 PC 上用 gcc 编译）
 *
 * 精简版（直道+转弯）：无状态机/契约层。转向对 track_info.error 做调度 Kp 的 PD；
 * 速度策略简化为：target = min(STRAIGHT_DUTY, DUTY_HARD_CAP) + 升/降斜坡。
 ********************************************************************************************************************/
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "control.h"

/*===================================================================================================================
 * 一、菜单可调定标（菜单直接编辑这些 volatile 全局）
 *-------------------------------------------------------------------------------------------------------------------
 * 上电初值取 config.h 的默认宏；menu_init() 之后若 Flash 有存档会覆盖它们。
 * 菜单只在“编辑结束”那一刻单次写入（16/32 位对齐写），本控制律每帧读取 —— 永不会看到半改值。
 * PC replay 不编译菜单（menu_config.c 不参与），这些量始终保持默认 → 离线回放逐位不变。
 *==================================================================================================================*/

volatile float   steer_kp_min       = KP_MIN;
volatile float   steer_kp_max       = KP_MAX;
volatile float   steer_kp_e_sat     = KP_E_SAT;
volatile uint8_t steer_use_const_kp = USE_CONST_KP;
volatile float   steer_kp_const     = KP_CONST;
volatile float   steer_kd           = KD;
volatile float   steer_d_filt_alpha = D_FILT_ALPHA;

/*===================================================================================================================
 * 二、内部状态（跨帧记忆）
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
    if (servo_raw < SERVO_MIN)
    {
        servo_raw = SERVO_MIN;
    }
    if (servo_raw > SERVO_MAX)
    {
        servo_raw = SERVO_MAX;
    }
    return (uint16_t)servo_raw;
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
    /*==================================== 转向 PD ====================================
     * 误差直接取加权中线误差；D 项跨帧记忆在 control_init()（解锁）时归零。 */
    int16_t error = ti->error;

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
        if (ratio > 1.0f)
        {
            ratio = 1.0f;
        }
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

    /*==================================== 速度（定速开环） ====================================*/
    uint16_t target = STRAIGHT_DUTY;
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
