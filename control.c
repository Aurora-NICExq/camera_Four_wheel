/* control.c - gain-scheduled PD steer + curve speed schedule */
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "control.h"

volatile float   steer_kp_min       = KP_MIN;
volatile float   steer_kp_max       = KP_MAX;
volatile float   steer_kp_e_sat     = KP_E_SAT;
volatile float   steer_kd           = KD;
volatile float   steer_d_filt_alpha = D_FILT_ALPHA;
volatile uint16_t curve_duty       = CURVE_DUTY;
volatile uint16_t straight_duty    = STRAIGHT_MAX_DUTY;
volatile uint16_t curve_temp_div    = CURVE_TEMP_DIV;
volatile int16_t straight_judge     = STRAIGHT_JUDGE;
volatile int16_t straight_judge_13  = STRAIGHT_JUDGE_13;
volatile uint8_t  drive_armed       = 0;
volatile uint8_t  drive_timed_out   = 0;
volatile uint16_t drive_stop_time_s = DRIVE_ARMED_TIMEOUT_S;
volatile uint16_t drive_duty_base   = STRAIGHT_DUTY;
volatile uint16_t control_duty_prev = 0;

static int16_t  g_prev_error;
static float    g_d_filt;
static uint16_t g_duty_now;
static uint8_t  g_slow_motor;
static uint8_t  g_exit_cnt;
static float    g_temp_hold;

static int16_t iabs16(int16_t v)
{
    return (v >= 0) ? v : (int16_t)(-v);
}

/* 近车端 track 行：TR_ROW(EIGHTN_START_ROW)，与 export_track 写入对齐 */
static uint8_t track_near_row(void)
{
    return (uint8_t)(IMG_H - 1u - EIGHTN_START_ROW);
}

static uint8_t track_far_row(const track_info_t *ti)
{
    return (uint8_t)(track_near_row() + ti->valid_rows - 1u);
}

static uint8_t track_mid_row(const track_info_t *ti)
{
    return (uint8_t)(track_near_row() + ti->valid_rows / 2u);
}

uint16_t control_servo_clamp(int32_t servo_raw)
{
    if (servo_raw < SERVO_MIN) servo_raw = SERVO_MIN;
    if (servo_raw > SERVO_MAX) servo_raw = SERVO_MAX;
    return (uint16_t)servo_raw;
}

/* 单侧严格直道：远/中/近三行边界共线 */
static uint8_t border_strict(const track_info_t *ti, uint8_t use_left)
{
    uint8_t far_r;
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

    near_r = track_near_row();
    far_r  = track_far_row(ti);
    mid_r  = track_mid_row(ti);

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

static uint8_t straight_flag_judge(const track_info_t *ti)
{
    if (ti->valid_rows < STRAIGHT_MIN_ROWS)
    {
        return 0u;
    }
    if (border_strict(ti, 1u) && border_strict(ti, 0u))
    {
        return 1u;
    }
    return 0u;
}

/* 采样窗内中线均值,跳过双边丢线行。
   空间平均只用同一帧的多行数据,降噪但不引入任何时间延迟 */
static uint8_t mid_window_avg(const track_info_t *ti, int16_t center, int16_t *out)
{
    int16_t lo = (int16_t)(center - CURV_HALF_WIN);
    int16_t hi = (int16_t)(center + CURV_HALF_WIN);
    int16_t r;
    int32_t sum = 0;
    uint8_t n = 0;

    if (lo < 1) lo = 1;
    if (hi > (int16_t)ti->valid_rows) hi = (int16_t)ti->valid_rows;

    for (r = lo; r <= hi; r++)
    {
        uint8_t tr = (uint8_t)r;
        if (ti->left_lost[tr] && ti->right_lost[tr])
        {
            continue; /* 双边丢线行的 mid 是初始化的假居中值 */
        }
        sum += (int32_t)ti->mid[tr];
        n++;
    }
    if (n < CURV_MIN_SAMPLES)
    {
        return 0u;
    }
    *out = (int16_t)(sum / (int32_t)n);
    return 1u;
}

/* 真曲率:中线二阶差分 / 采样间距²。
   对任意倾斜的直线恒为 0(车身偏航不再被误判成弯道),
   除以间距² 后读数与视野长短无关 */
static int16_t track_curvature(const track_info_t *ti, uint8_t *ok)
{
    int16_t base = (int16_t)track_near_row();
    int16_t lo;
    int16_t hi;
    int16_t h;
    int16_t v_near;
    int16_t v_mid;
    int16_t v_far;
    int32_t d2;

    *ok = 0u;
    if (ti->valid_rows < (uint8_t)(CURV_ROW_NEAR + CURV_HALF_WIN + 2 * CURV_MIN_SPAN))
    {
        return 0;
    }

    lo = (int16_t)(base + CURV_ROW_NEAR);
    hi = (int16_t)(base + (int16_t)ti->valid_rows - 1 - CURV_HALF_WIN);
    h  = (int16_t)((hi - lo) / 2);
    if (h < CURV_MIN_SPAN)
    {
        return 0;
    }

    if (!mid_window_avg(ti, lo, &v_near) ||
        !mid_window_avg(ti, (int16_t)(lo + h), &v_mid) ||
        !mid_window_avg(ti, (int16_t)(lo + 2 * h), &v_far))
    {
        return 0;
    }

    *ok = 1u;
    d2 = (int32_t)v_far - 2 * (int32_t)v_mid + (int32_t)v_near;
    return (int16_t)((d2 * CURV_SCALE) / ((int32_t)h * (int32_t)h));
}

/* temp = |曲率| / Curv Div,上限 1;非对称保持:升立即、降缓慢 */
static float curve_temp(const track_info_t *ti, int16_t *curv_out)
{
    uint8_t ok;
    int16_t curv = track_curvature(ti, &ok);
    float temp;

    *curv_out = ok ? curv : 0;

    if (!ok)
    {
        temp = 1.0f; /* 看不清前方就按最保守处理,直接给弯道速度 */
    }
    else
    {
        uint16_t div = curve_temp_div;
        if (div == 0u) div = 1u;
        temp = (float)iabs16(curv) / (float)div;
        if (temp > 1.0f) temp = 1.0f;
    }

    /* 弯度上升立即生效,下降缓慢回落:S 弯与复合弯的中间段不会误加速 */
    if (temp > g_temp_hold)
    {
        g_temp_hold = temp;
    }
    else
    {
        g_temp_hold += CURVE_TEMP_FALL * (temp - g_temp_hold);
    }
    return g_temp_hold;
}

/* 有效行数限速(master cap_rows 通道):入弯口视野塌缩早于近端误差出现,
   按可见行数封顶目标占空比 */
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
    g_slow_motor = 1u;
    g_exit_cnt   = 0;
    g_temp_hold  = 0.0f;
}

void control_reset(void)
{
    control_init();
    control_duty_prev = 0;
}

void control_duty_reset(void)
{
    g_duty_now   = 0;
    g_slow_motor = 1u;
}

void control_update(const track_info_t *ti, control_out_t *out)
{
    int16_t error = ti->error;
    float speed_f;
    float temp;
    int16_t curv;
    uint16_t target;
    uint8_t straight;

    float d_raw = (float)(error - g_prev_error);
    g_prev_error = error;
    g_d_filt += steer_d_filt_alpha * (d_raw - g_d_filt);

    float e_abs = (error >= 0) ? (float)error : (float)(-error);
    float ratio = e_abs / steer_kp_e_sat;
    if (ratio > 1.0f) ratio = 1.0f;
    float kp = steer_kp_min + (steer_kp_max - steer_kp_min) * ratio * ratio;

    int32_t servo_raw = SERVO_CENTER
                      + (int32_t)(SERVO_DIR * (kp * (float)error + steer_kd * g_d_filt));

    out->servo_pwm = control_servo_clamp(servo_raw);

    straight = straight_flag_judge(ti);
    temp     = curve_temp(ti, &curv);
    out->curv = curv;
    out->temp_pct = (uint16_t)(temp * 100.0f);

    /* 出弯确认:误差连续收敛若干帧后才允许直道加速 */
    if (iabs16(error) <= EXIT_ERR_MAX && ti->valid_rows >= EXIT_ROWS_MIN &&
        ti->both_lost_rows <= EXIT_MAX_BOTH_LOST)
    {
        if (g_exit_cnt < 255u)
        {
            g_exit_cnt++;
        }
    }
    else
    {
        g_exit_cnt = 0;
    }

    speed_f = (float)drive_duty_base;
    if (speed_f > (float)curve_duty)
    {
        speed_f -= (speed_f - (float)curve_duty) * temp;
    }
    if (straight && g_exit_cnt >= EXIT_CONFIRM_FRAMES &&
        (float)straight_duty > speed_f)
    {
        speed_f = (float)straight_duty;
    }

    {
        uint16_t cap = rows_duty_cap(ti->valid_rows);
        out->rows_cap = cap;
        if (speed_f > (float)cap)
        {
            speed_f = (float)cap;
        }
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
    out->straight   = straight;
}
