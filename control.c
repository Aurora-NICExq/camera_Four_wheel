/*********************************************************************************************************************
 * 模块：control.c — 转向 PD + 滑行式开环速度策略（纯逻辑层，可在 PC 上用 gcc 编译）
 *
 * 依赖：<stdint.h> + config.h/image.h/fsm.h/control.h —— 不含任何 MCU / 逐飞头文件。
 * 浮点使用范围：仅每帧一次的控制数学（Kp 调度、D 滤波）；逐像素路径在 image.c，全整数。
 *
 * 速度策略核心事实（决定一切阈值的取法）：
 *   无编码器 + 正转单向驱动 → 减速只能靠滑行。切占空比之后什么都做不了，只能等车自己慢下来。
 *   因此："什么时候收油"比"给多大油"重要一个数量级 —— 两张减速表是全项目最重要的整定值，
 *   必须锚定 README《滑行特性标定》的实测结果，绝不许拍脑袋上调。
 ********************************************************************************************************************/
#include <stdint.h>
#include "config.h"
#include "image.h"
#include "fsm.h"
#include "control.h"

/*===================================================================================================================
 * 一、内部状态（跨帧记忆）
 *==================================================================================================================*/

static int16_t  g_prev_error;       /* 上一帧误差（D 项差分基准）                                */
static float    g_d_filt;           /* 滤波后的 D 项（EMA）                                      */
static uint16_t g_duty_now;         /* 斜坡后的当前占空比（真实下发值）                          */
static uint8_t  g_exit_cnt;         /* 弯道出口特征连续确认计数                                  */
static uint8_t  g_boost_cnt;        /* 完美直道持续计数                                          */
static uint8_t  g_boost_on;         /* 1 = boost 生效中                                          */
static uint16_t g_servo_now;        /* 舵机输出限速后的当前命令                                  */

/*===================================================================================================================
 * 二、内部工具
 *==================================================================================================================*/

/*-------------------------------------------------------------------------------------------------------------------
 * control_servo_clamp — 最终机械限位钳制（全工程唯一实现）
 * 任何舵机命令的必经之路；motor.c 写硬件前也调用它做第二道钳制。
 * 超出机械极限的命令会打坏舵机齿轮 —— 这里没有任何例外分支。
 *------------------------------------------------------------------------------------------------------------------*/
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

/*-------------------------------------------------------------------------------------------------------------------
 * steer_error_from_contract — 按契约表指定的转向源生成本帧误差（像素）
 * NORMAL/CROSS：加权中线误差；环内：单边 ± 入环前学习的半宽；出环：固定偏置。
 * 单边巡线取近行带 [CURV_NEAR_ROW_LO, CURV_NEAR_ROW_HI] 平均：近端边线像素分辨率最高、
 * 受环内伪边干扰最小。
 *------------------------------------------------------------------------------------------------------------------*/
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

/*-------------------------------------------------------------------------------------------------------------------
 * lut_lookup_ascend — 分段查表（输入升序档位，取第一个 x < 档位的输出；超出最后档给末值）
 * 两张减速表共用。故意用查表而不是线性公式：滑行减速是强非线性过程，标定数据长什么样，
 * 表就长什么样，不强行拟合。
 *------------------------------------------------------------------------------------------------------------------*/
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

/*===================================================================================================================
 * 三、对外接口
 *==================================================================================================================*/

void control_init(void)
{
    g_prev_error = 0;
    g_d_filt     = 0.0f;
    g_duty_now   = 0;       /* 从 0 起步：解锁后的软启动就是升斜坡从 0 爬升，无需单独机制 */
    g_exit_cnt   = 0;
    g_boost_cnt  = 0;
    g_boost_on   = 0;
    g_servo_now  = SERVO_CENTER;
}

/*-------------------------------------------------------------------------------------------------------------------
 * control_update — 每帧一次（fsm_update 之后）。见 control.h 接口说明。
 *------------------------------------------------------------------------------------------------------------------*/
void control_update(const track_info_t *ti, uint8_t armed, control_out_t *out)
{
    const state_contract_t *ct = fsm_contract();

    /*==================================== 转向 PD ====================================*/
    int16_t error = steer_error_from_contract(ti, ct);

    /* D 项防踢：状态切换帧转向源跳变（中线→单边→偏置），若沿用旧 prev_error，
     * 差分会出现一根与车辆运动无关的巨大尖峰直接打进舵机。该帧强制 D=0。 */
    if (fsm_state_just_entered())
    {
        g_prev_error = error;
        g_d_filt     = 0.0f;
    }

    float d_raw = (float)(error - g_prev_error);
    g_prev_error = error;
    /* 像素量化让原始差分呈锯齿状，EMA 把 D 项从"噪声放大器"拉回"阻尼项" */
    g_d_filt += D_FILT_ALPHA * (d_raw - g_d_filt);

#if USE_CONST_KP
    float kp = KP_CONST;
#else
    /* Kp 随 |误差| 二次调度：e=0 附近 KP_MIN（boost 直道不画龙），
     * |e|≥KP_E_SAT 达到 KP_MAX（弯道打舵权威）。二次而非线性：小误差区更平缓。 */
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

    /* 先机械钳制，再限制单帧变化。即使图像候选在两条白支路间切换，也不能一帧反打全舵程。 */
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

    /*==================================== 速度（开环滑行式） ====================================*/
    static const int16_t  rows_bins[ROWS_DUTY_TABLE_LEN] = ROWS_DUTY_TABLE_ROWS;
    static const uint16_t rows_duty[ROWS_DUTY_TABLE_LEN] = ROWS_DUTY_TABLE_DUTY;
    static const int16_t  curv_bins[CURV_DUTY_TABLE_LEN] = CURV_DUTY_TABLE_CURV;
    static const uint16_t curv_duty[CURV_DUTY_TABLE_LEN] = CURV_DUTY_TABLE_DUTY;

    /* 1. 行数减速表：看不远 → 必须慢到能在可见距离内滑到弯速 */
    uint16_t cap_rows = lut_lookup_ascend(ti->valid_rows, rows_bins, rows_duty, ROWS_DUTY_TABLE_LEN);

    /* 2. 曲率减速表：看得远但前方是弯 → 同样提前收油 */
    int16_t curv_abs = (ti->curvature >= 0) ? ti->curvature : (int16_t)(-ti->curvature);
    uint16_t cap_curv = lut_lookup_ascend(curv_abs, curv_bins, curv_duty, CURV_DUTY_TABLE_LEN);

    /* 3. 转向减速：舵角越大油门上限越低（大舵角 + 大油门 = 甩尾/推头）。
     * 从硬上限起算：舵机在中位时本项不构成限制。 */
    int32_t steer_off = (int32_t)out->servo_pwm - SERVO_CENTER;
    if (steer_off < 0)
    {
        steer_off = -steer_off;
    }
    int32_t cap_steer_i = (int32_t)DUTY_HARD_CAP - (steer_off * STEER_DUTY_SLOPE_NUM) / STEER_DUTY_SLOPE_DEN;
    uint16_t cap_steer = (cap_steer_i < MIN_TURN_DUTY) ? MIN_TURN_DUTY : (uint16_t)cap_steer_i;

    /* 4. 长直道 boost 判定：完美直道持续确认后把"基准"从 STRAIGHT_DUTY 抬到 BOOST_DUTY；
     * 远端一见曲率立即退出（退出阈值 BOOST_EXIT_CURV 低于减速表首档 —— 迟滞）。
     * boost 上限的物理含义：从 BOOST_DUTY 滑到 MIN_TURN_DUTY 的距离必须 < 视觉可见距离，
     * 上调 BOOST_DUTY 前必须重做滑行标定 (c) 步 —— 否则第一个弯就冲出去。 */
    int16_t err_abs = (ti->error >= 0) ? ti->error : (int16_t)(-ti->error);
    if (err_abs <= BOOST_ERR_MAX && ti->valid_rows >= BOOST_ROWS_MIN && curv_abs <= EXIT_CURV_MAX &&
        ti->both_lost_rows <= BOOST_MAX_BOTH_LOST &&
        fsm_state() == ST_NORMAL)
    {
        if (g_boost_cnt < 255u)
        {
            g_boost_cnt++;
        }
    }
    else
    {
        g_boost_cnt = 0;
    }
    if (curv_abs > BOOST_EXIT_CURV || fsm_state() != ST_NORMAL)
    {
        g_boost_on = 0;         /* 远端出现弯道特征：立刻放弃 boost，开始滑行 */
    }
    else if (g_boost_cnt >= BOOST_CONFIRM_FRAMES)
    {
        g_boost_on = 1;
    }

    /* 5. 综合：目标 = min(基准, 各减速表, 契约表状态上限)。
     * 注意两张表的最高档必须 ≥ BOOST_DUTY，否则 boost 在结构上永远无法生效（见 config.h 注释）。 */
    uint16_t target = g_boost_on ? BOOST_DUTY : STRAIGHT_DUTY;
    if (cap_rows  < target) { target = cap_rows;  }
    if (cap_curv  < target) { target = cap_curv;  }
    if (cap_steer < target) { target = cap_steer; }

    /* 环岛已经由状态机完成进入确认和阶段管理，契约表的 2800/3000/3000 就是该阶段的保守速度。
     * 若图像健康且前瞻充足，允许状态契约接管，不再被通用曲率表长期压回 MIN_TURN_DUTY。
     * 图像失效或前瞻不足时不覆盖，继续使用上面的通用保守结果。 */
    fsm_state_t speed_state = fsm_state();
    uint8_t severe_image = 0u;
    uint8_t ring_speed_override = 0u;
    if ((speed_state == ST_RING_PRE || speed_state == ST_RING_IN || speed_state == ST_RING_EXIT) &&
        ti->valid_rows >= ELEMENT_SPEED_MIN_ROWS && !image_track_invalid(ti, &severe_image))
    {
        target = ct->duty_cap;
        ring_speed_override = 1u;
    }

    /* 6. 弯道出口再加速确认：特征满足则允许沿升斜坡回升（出弯早晚比直道极速更影响圈速）。
     * 未确认时把目标压在当前值 —— 防止弯心里行数瞬时回升引起的"油门抽动"。
     * 双边丢失行多 = 前瞻里有大段无边白区（十字/环口/宽路口），valid_rows 只代表可行驶
     * 距离而非边线可见距离，此时禁止确认再加速。 */
    if (err_abs <= EXIT_ERR_MAX && ti->valid_rows >= EXIT_ROWS_MIN && curv_abs <= EXIT_CURV_MAX &&
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
    if (!ring_speed_override && target > g_duty_now && g_exit_cnt < EXIT_CONFIRM_FRAMES)
    {
        /* 想加速？先连续 EXIT_CONFIRM_FRAMES 帧证明真的出弯了。
         * 但永远允许爬到 MIN_TURN_DUTY：它按定义是"最急弯也安全"的占空比 ——
         * 否则弯中重新解锁的车会因为误差始终偏大而永远趴着不动（死锁）。 */
        uint16_t floor_duty = (g_duty_now > MIN_TURN_DUTY) ? g_duty_now : MIN_TURN_DUTY;
        if (target > floor_duty)
        {
            target = floor_duty;
        }
    }

    /* 7. 契约表状态上限 + 全局硬上限（放在最后：任何机制都压不过它们） */
    if (target > ct->duty_cap)
    {
        target = ct->duty_cap;
    }
    if (target > DUTY_HARD_CAP)
    {
        target = DUTY_HARD_CAP;
    }

    out->duty_target = target;

    /* 8. 斜坡：降默认不限（切油=开始滑行，无反拖不会抱死）；升小步长（防打滑，兼软启动） */
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

    /* 9. 武装门闸：未解锁/失控保护 → 占空比强制 0（舵机保持随动便于推车对中）。
     * DEBUG_NO_DRIVE 的强制断油在 main.c 的门闸处执行，本模块照常计算便于观察决策值。 */
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
