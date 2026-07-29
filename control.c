/* control.c - 单套 PD 转向 + 单一占空比 */
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "control.h"

volatile float    steer_kp           = KP;
volatile float    steer_kd           = KD;
volatile float    steer_d_filt_alpha = D_FILT_ALPHA;
volatile uint8_t  drive_armed        = 0;
volatile uint8_t  drive_timed_out    = 0;
volatile uint16_t drive_stop_time_s  = DRIVE_ARMED_TIMEOUT_S;
volatile uint16_t drive_duty_base    = STRAIGHT_DUTY;
volatile uint16_t control_duty_prev  = 0;

static int16_t  g_prev_error;
static float    g_d_filt;
static uint16_t g_duty_now;

uint16_t control_servo_clamp(int32_t servo_raw)
{
    if (servo_raw < SERVO_MIN) servo_raw = SERVO_MIN;
    if (servo_raw > SERVO_MAX) servo_raw = SERVO_MAX;
    return (uint16_t)servo_raw;
}

/* 有效行数安全网:视野严重劣化时兜底降速。
   这不是"直道/弯道"调度,而是对图像质量的保护,故予以保留 */
static uint16_t rows_duty_cap(uint8_t rows)
{
    static const uint8_t  bins[ROWS_CAP_TABLE_LEN] = ROWS_CAP_BINS;
    static const uint16_t caps[ROWS_CAP_TABLE_LEN] = ROWS_CAP_DUTY;
    uint8_t i;

    for (i = 0; i < ROWS_CAP_TABLE_LEN; i++)
    {
        if (rows < bins[i])
        {
            return caps[i];
        }
    }
    return DUTY_HARD_CAP;
}

void control_init(void)
{
    g_prev_error = 0;
    g_d_filt     = 0.0f;
    g_duty_now   = 0;
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
    int16_t  error = ti->error;
    float    speed_f;
    uint16_t target;
    float    d_raw;
    int32_t  servo_raw;

    /* 单套 PD:恒定比例增益,不随误差调度 */
    d_raw = (float)(error - g_prev_error);
    g_prev_error = error;
    g_d_filt += steer_d_filt_alpha * (d_raw - g_d_filt);

    servo_raw = SERVO_CENTER
              + (int32_t)(SERVO_DIR * (steer_kp * (float)error + steer_kd * g_d_filt));
    out->servo_pwm = control_servo_clamp(servo_raw);

    /* 速度:单一 Duty,不再区分直道与弯道 */
    speed_f = (float)drive_duty_base;
    {
        uint16_t cap = rows_duty_cap(ti->valid_rows);
        out->rows_cap = cap;
        if (speed_f > (float)cap)
        {
            speed_f = (float)cap;
        }
    }

    if (speed_f < 0.0f)
    {
        speed_f = 0.0f;
    }
    target = (uint16_t)speed_f;
    if (target > DUTY_HARD_CAP) target = DUTY_HARD_CAP;

    out->duty_target = target;

    if (target > g_duty_now)
    {
        uint16_t step = (uint16_t)(target - g_duty_now);
        g_duty_now += (step > DUTY_SLEW_UP) ? DUTY_SLEW_UP : step;
    }
    else
    {
        g_duty_now = target; /* 减速不限幅:目标降低立即跟进 */
    }

    out->duty       = g_duty_now;
    out->error_used = error;
}
