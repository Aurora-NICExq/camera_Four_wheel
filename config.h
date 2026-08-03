/* config.h */
#ifndef CONFIG_H
#define CONFIG_H

#define IMG_W (188)
#define IMG_H (120)
#define IMG_CENTER (94)

#define DRIVE_LAUNCH_DELAY_S (2)
#define DRIVE_ARMED_TIMEOUT_S (30)
#define DRIVE_LAUNCH_DELAY_US ((uint32_t)DRIVE_LAUNCH_DELAY_S * 1000000u)
#define DRIVE_DT_CLAMP_US (200000u)
#define DRIVE_DT_NOMINAL_US (20000u)

#define FIXED_THRESHOLD (128)
#define OTSU_ROW_STEP (2)
#define OTSU_COL_STEP (2)
#define OTSU_THRESHOLD_MIN (40)
#define OTSU_THRESHOLD_MAX (200)

// 前瞻相关参数
//
// 单行前瞻:菜单 Look Far 直接就是瞄准行(tr 坐标,越大越远;tr=1
// 是画面最下面一行)。 该行双边丢线时向近端滑到第一条有效行,aim_row
// 回报实际行号;0 = 无有效行。 丢线保护看 aim_row==0,帧数阈值
// FAILSAFE_FRAMES(写死,见 3bc4cc6)。

#define STEER_LOOK_FAR_DEFAULT (115)
#define STEER_LOOK_FAR_MAX (IMG_H - 1)
#if (STEER_LOOK_FAR_DEFAULT > STEER_LOOK_FAR_MAX) ||                           \
    (STEER_LOOK_FAR_DEFAULT < 1)
#error "STEER_LOOK_FAR_DEFAULT out of range"
#endif
#define ERR_HOLD_MAX_FRAMES (20)

/* 直道/弯道判别:拟合中线相对屏幕中线的方差(实验中,未上车标定)
 *
 *   var = Σ(mid[tr] - IMG_CENTER)² / n   单位:像素²
 *
 * 窗口 = 转向用的同一个前瞻窗口 tr ∈ [1, Look Far],刻意不另开一个
 * 行范围参数:判据要说的是"你正在拿来转向的那段中线弯不弯"。
 * 只统计非双边丢线行,n(= mid_var_rows)一并输出 —— 参与行数会随视野
 * 剧烈变化,不报出来就是不可观测的降级(R6)。
 *
 * 同时算一个相对样本自身均值的方差 mid_var_ac(= var - mean²),只上屏
 * 不参与判据,用途是回答"var 大到底是因为弯,还是因为车横偏":
 *   直道居中     var 小,   var_ac 小
 *   直道横偏 10px var≈100, var_ac 小     ← 若实测常见此形态,判据应改用 var_ac
 *   真弯道       var 大,   var_ac 也大
 *
 * 合成中线跑出来的量级(Look Far=115,窗口 115 行):
 *   直道居中          VAR    0   VRM   0
 *   直道横偏 10px     VAR  100   VRM   0
 *   中线扫到 -60px    VAR 1186   VRM 345
 *   中线扫到 -90px    VAR 2692   VRM 756
 *
 * ★ 阈值不跨 Look Far 通用:同一条"扫到 -60px"的弯,Look Far=40 时
 *   VAR 只有 140(窗口短 → 远端大偏差根本没进统计)。改 Look Far 必须
 *   重新标 Var Th。三档预设的 Look Far 是 75,默认值是 115,两者的
 *   Var Th 不是一个数 —— 这是把窗口挂在 Look Far 上换来的代价,
 *   换来的是不用再多一个"方差窗口"参数。若实测觉得这个耦合更烦,
 *   下一步就把窗口写死成固定行区间,而不是给它加系数。
 *
 * CURVE_VAR_TH_DEFAULT 取 400 是占位值,不是标定值(≈ 平均偏离 20px)。
 * 上车先在 Camera 页读直道/弯道各自的 VAR,再决定阈值 —— 在能回答
 * "直道跑到多少、弯道跑到多少"之前这个数没有意义(R2),所以它进菜单
 * 的唯一理由就是做这次实验。
 * 判别结果 is_curve 当前不接入任何控制,只上屏。
 * CURVE_VAR_MIN_ROWS:参与行数不足时本帧不判别(is_curve 置 0)。 */
#define CURVE_VAR_TH_DEFAULT (400)
#define CURVE_VAR_MIN_ROWS (10)
#define CURVE_VAR_MAX (IMG_CENTER * IMG_CENTER)

#define EIGHTN_START_ROW (IMG_H - 2)
#define EIGHTN_BORDER_MIN (1)
#define EIGHTN_BORDER_MAX (IMG_W - 2)
#define EIGHTN_MAX_POINTS (IMG_H * 3)
#define EIGHTN_FILTER_SUM_MAX (255 * 5)
#define EIGHTN_FILTER_SUM_MIN (255 * 2)
#define EIGHTN_MEET_DIST (2)
#define EIGHTN_EDGE_LOST_MARGIN (2)

#define EIGHTN_CROSS_SLOPE_BACK (15)
#define EIGHTN_CROSS_SLOPE_NEAR (5)
#define EIGHTN_CROSS_CORNER_L (4)
#define EIGHTN_CROSS_CORNER_R (IMG_W - 4)
#define EIGHTN_CROSS_BREAK_DROW (15)
#define EIGHTN_CROSS_OPEN_WIDTH (140)
#define EIGHTN_CROSS_OPEN_ROW_MAX (IMG_H - 8)

/* 向量法拐点检测(取代原来的 dir 序列 4,4,...,6,6,6)。
   沿爬线取 P0=pts[i]、P1=pts[i+K]、P2=pts[i+2K],P1 是候选角点,
   判据全部作用在"进入向量 a=P1-P0"和"离开向量 b=P2-P1"的几何量上,
   不依赖方向码在几个固定偏移上的精确取值,边线抖一个像素不会整帧漏检。
   VEC_K    三点间隔(爬线点数)。太小对边缘噪声敏感,太大漏掉小角。预期 4~8。
   VEC_FLAT K 步内 y 变化多少算"竖直":进入段须 |dy|<=FLAT(判定为水平),
            离开段须 dy<=-FLAT(判定为向上)。K 步最多走 K 行,
            故必须 FLAT < K,否则两个判据同时无解、永远检不出拐点。 */
#define EIGHTN_CROSS_VEC_K (6)
#define EIGHTN_CROSS_VEC_FLAT (4)
#if (EIGHTN_CROSS_VEC_K < 2) ||                                                \
    (EIGHTN_CROSS_VEC_FLAT >= EIGHTN_CROSS_VEC_K) ||                           \
    (EIGHTN_CROSS_VEC_FLAT < 1)
#error "EIGHTN_CROSS_VEC_FLAT must satisfy 1 <= FLAT < VEC_K"
#endif

// 舵机参数

#define SERVO_PWM_HZ (50)
#define SERVO_CENTER (705)
#define SERVO_MIN (629)
#define SERVO_MAX (781)
#define SERVO_DIR (-1)

#define KP (1.20f)
#define KD (1.49f)
#define D_FILT_ALPHA (0.4f)

#define STRAIGHT_DUTY (2100)
#define DUTY_HARD_CAP (6000)
#define DUTY_SLEW_UP (120)

#define FAILSAFE_FRAMES (2)

#define MOTOR_PWM_FREQ (17000)

#define KEY_SCAN_PERIOD_MS (5)
#define KEY_DEBOUNCE_MS (20)
#define KEY_DEBOUNCE_COUNT (KEY_DEBOUNCE_MS / KEY_SCAN_PERIOD_MS)
#define KEY_LONG_PRESS_MS (1000)
#define KEY_REPEAT_MS (80)

#define MOTOR_TEST_DUTY (2000)

// 保存的三版参数

// 这个他妈测一次就出来了，前瞻是对的
#define PRESET_LOW_KP (1.45f)
#define PRESET_LOW_KD (1.00f)
#define PRESET_LOW_D_ALPHA (0.40f)
#define PRESET_LOW_THRESHOLD (255)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_LOOK_FAR (75)
#define PRESET_LOW_DUTY (2750)
#define PRESET_LOW_STOP_TIME (25)

#define PRESET_MID_KP (1.59f)
#define PRESET_MID_KD (0.80f)
#define PRESET_MID_D_ALPHA (0.40f)
#define PRESET_MID_THRESHOLD (255)
#define PRESET_MID_CROSS_FILL (1)
#define PRESET_MID_LOOK_FAR (75)
#define PRESET_MID_DUTY (2950)
#define PRESET_MID_STOP_TIME (22)

#define PRESET_HIGH_KP (KP)
#define PRESET_HIGH_KD (KD)
#define PRESET_HIGH_D_ALPHA (D_FILT_ALPHA)
#define PRESET_HIGH_THRESHOLD (0)
#define PRESET_HIGH_CROSS_FILL (1)
#define PRESET_HIGH_LOOK_FAR (STEER_LOOK_FAR_DEFAULT)
#define PRESET_HIGH_DUTY (STRAIGHT_DUTY)
#define PRESET_HIGH_STOP_TIME (DRIVE_ARMED_TIMEOUT_S)

#endif /* CONFIG_H */
