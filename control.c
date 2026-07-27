/* control.c - 丑牛 CCD 算法：servoPID_ccd + steering_type 速度调度 */
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "control.h"

volatile float    steer_kp           = SERVO_P;
volatile float    steer_kp2          = SERVO_P2;
volatile float    steer_ki          = SERVO_I;
volatile float    steer_kd           = SERVO_D;
volatile float    steer_ka           = SERVO_A;
volatile float    steer_up           = STEER_UP_DUTY;
volatile float    speed_up           = SPEED_UP_DUTY;
volatile int16_t  straight_judge     = STRAIGHT_JUDGE;
volatile int16_t  straight_judge_13  = STRAIGHT_JUDGE_13;
volatile uint8_t  drive_armed        = 0;
volatile uint16_t drive_duty_base    = EX_SPEED_DUTY;
volatile uint16_t control_duty_prev  = 0;

static int16_t  g_servo_error_pre;
static uint16_t g_duty_now;
static uint8_t  g_slow_motor;

static int16_t iabs16(int16_t v)
{
    return (v >= 0) ? v : (int16_t)(-v);
}

static float f_abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

uint16_t control_servo_clamp(int32_t servo_raw)
{
    if (servo_raw < SERVO_MIN) servo_raw = SERVO_MIN;
    if (servo_raw > SERVO_MAX) servo_raw = SERVO_MAX;
    return (uint16_t)servo_raw;
}

/* 丑牛 change_error(1)：中段 CCD 误差 → 近端 track 行中线 */
static int16_t steer_error_mid(const track_info_t *ti)
{
    uint8_t near_r;

    if (ti->valid_rows == 0u)
    {
        return 0;
    }
    near_r = (uint8_t)(ti->valid_rows / 2u);
    if (near_r >= ti->valid_rows)
    {
        near_r = (uint8_t)(ti->valid_rows - 1u);
    }
    return (int16_t)ti->mid[near_r] - IMG_CENTER;
}

/* 丑牛 straight() 单侧严格直道：l_strict / r_strict */
static uint8_t border_strict(const track_info_t *ti, uint8_t use_left)
{
    uint8_t far_r = 0;
    uint8_t mid_r;
    uint8_t near_r;
    int16_t v_far;
    int16_t v_mid;
    int16_t v_near;
    int16_t tem;
    int16_t tem1;

    if (ti->valid_rows < 6u)
    {
        return 0u;
    }

    near_r = (uint8_t)(ti->valid_rows - 1u);
    mid_r  = (uint8_t)(ti->valid_rows / 2u);

    if (use_left)
    {
        if (ti->left_lost[far_r] || ti->left_lost[mid_r] || ti->left_lost[near_r])
        {
            return 0u;
        }
        v_far  = (int16_t)ti->left[far_r];
        v_mid  = (int16_t)ti->left[mid_r];
        v_near = (int16_t)ti->left[near_r];
    }
    else
    {
        if (ti->right_lost[far_r] || ti->right_lost[mid_r] || ti->right_lost[near_r])
        {
            return 0u;
        }
        v_far  = (int16_t)ti->right[far_r];
        v_mid  = (int16_t)ti->right[mid_r];
        v_near = (int16_t)ti->right[near_r];
    }

    tem  = iabs16((int16_t)(v_mid - v_near)) + iabs16((int16_t)(v_far - v_mid));
    tem1 = iabs16((int16_t)(v_far - v_near));

    if (tem <= straight_judge && tem1 <= straight_judge)
    {
        return 1u;
    }
    if (tem1 >= straight_judge_13)
    {
        return 0u;
    }
    return 0u;
}

/* 丑牛 speed_up_judge() */
static uint8_t straight_flag_judge(const track_info_t *ti)
{
    if (border_strict(ti, 1u) && border_strict(ti, 0u))
    {
        return 1u;
    }
    return 0u;
}

/* 丑牛 temp = |c2.ipm_middle - c1.ipm_middle| / 30，上限 1 */
static float curve_temp(const track_info_t *ti)
{
    uint8_t near_r;
    uint8_t far_r;
    int16_t diff;
    float temp;

    if (ti->valid_rows < 2u)
    {
        return 1.0f;
    }

    near_r = (uint8_t)(ti->valid_rows - 1u);
    far_r  = (uint8_t)(ti->valid_rows / 2u);
    diff   = (int16_t)ti->mid[far_r] - (int16_t)ti->mid[near_r];
    temp   = (float)iabs16(diff) / (float)CURVE_TEMP_DIV;
    if (temp > 1.0f)
    {
        temp = 1.0f;
    }
    return temp;
}

/* 丑牛 servoPID_ccd；gyro=0 时退化为 P+P2+D */
static uint16_t servo_pid_ccd(float error, float gyro)
{
    float angle_now;
    static float gyro_last;

    angle_now = 0.8f * gyro + 0.2f * gyro_last;
    gyro_last = angle_now;

    {
        float out_f = (float)SERVO_CENTER
                    + (float)SERVO_DIR * (
                          steer_kp  * error
                        + steer_kp2 * error * f_abs(error)
                        + steer_ki
                        + steer_kd  * (error - (float)g_servo_error_pre)
                        - steer_ka  * angle_now);
        g_servo_error_pre = (int16_t)error;
        return control_servo_clamp((int32_t)out_f);
    }
}

void control_init(void)
{
    g_servo_error_pre = 0;
    g_duty_now        = 0;
    g_slow_motor      = 1u;
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
    int16_t error;
    float speed_f;
    float temp;
    uint16_t target;
    uint8_t straight;

    error = steer_error_mid(ti);
    out->servo_pwm = servo_pid_ccd((float)error, 0.0f);

    straight = straight_flag_judge(ti);
    temp     = curve_temp(ti);

    speed_f = (float)drive_duty_base;
    speed_f += steer_up * (1.0f - temp);
    if (straight)
    {
        speed_f += speed_up;
    }

    if (g_slow_motor)
    {
        int32_t diff = (int32_t)speed_f - (int32_t)g_duty_now;
        if (diff < 0) diff = -diff;
        if (diff <= SLOW_MOTOR_STEP || g_duty_now >= (uint16_t)speed_f)
        {
            g_slow_motor = 0u;
        }
        else
        {
            speed_f = (float)g_duty_now + speed_f / 4.0f;
        }
    }

    if (speed_f < 0.0f)
    {
        speed_f = 0.0f;
    }
    target = (uint16_t)speed_f;
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

    out->duty       = g_duty_now;
    out->error_used = error;
}
