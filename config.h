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

// 双核分工:CPU0 = 相机+图像+控制+电机,CPU1 = 按键+菜单+屏幕(见 shared.h)。
// CPU1 卡死不再拖停主循环,所以 CPU0 要盯它的心跳。阈值必须远大于 CPU1
// 画一帧灰度图的耗时(十几毫秒量级),否则一进 Camera 页就误锁死。
#define CPU1_ALIVE_TIMEOUT_US (500000u)

#define FIXED_THRESHOLD (128)
#define OTSU_ROW_STEP (2)
#define OTSU_COL_STEP (2)
#define OTSU_THRESHOLD_MIN (40)
#define OTSU_THRESHOLD_MAX (200)

// 前瞻相关参数
//
// 单行前瞻:菜单 Look Far 直接就是瞄准行(tr 坐标,越大越远;tr=1 是画面最下面一行)。
// 该行双边丢线时向近端滑到第一条有效行,aim_row 回报实际行号;0 = 无有效行。
// 丢线保护看 aim_row==0,帧数阈值菜单 Lost Fr。

#define STEER_LOOK_FAR_DEFAULT (115)
#define STEER_LOOK_FAR_MAX (IMG_H - 1)
#if (STEER_LOOK_FAR_DEFAULT > STEER_LOOK_FAR_MAX) ||                           \
    (STEER_LOOK_FAR_DEFAULT < 1)
#error "STEER_LOOK_FAR_DEFAULT out of range"
#endif
#define ERR_HOLD_MAX_FRAMES (20)

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

#define FAILSAFE_FRAMES_DEFAULT (10)
#define FAILSAFE_FRAMES_MIN (1)
#define FAILSAFE_FRAMES_MAX (60)

#define MOTOR_PWM_FREQ (17000)

#define KEY_SCAN_PERIOD_MS (5)
#define KEY_DEBOUNCE_MS (20)
#define KEY_DEBOUNCE_COUNT (KEY_DEBOUNCE_MS / KEY_SCAN_PERIOD_MS)
#define KEY_LONG_PRESS_MS (1000)
#define KEY_REPEAT_MS (80)

#define MOTOR_TEST_DUTY (2000)

// 保存的三版参数

// 这个他妈测一次就出来了，前瞻是对的
#define PRESET_LOW_KP (1.25f)
#define PRESET_LOW_KD (1.20f)
#define PRESET_LOW_D_ALPHA (0.40f)
#define PRESET_LOW_THRESHOLD (0)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_LOOK_FAR (75)
#define PRESET_LOW_DUTY (2500)
#define PRESET_LOW_STOP_TIME (25)

#define PRESET_MID_KP (1.65f)
#define PRESET_MID_KD (1.10f)
#define PRESET_MID_D_ALPHA (0.40f)
#define PRESET_MID_THRESHOLD (0)
#define PRESET_MID_CROSS_FILL (1)
#define PRESET_MID_LOOK_FAR (80)
#define PRESET_MID_DUTY (2600)
#define PRESET_MID_STOP_TIME (25)

#define PRESET_HIGH_KP (KP)
#define PRESET_HIGH_KD (KD)
#define PRESET_HIGH_D_ALPHA (D_FILT_ALPHA)
#define PRESET_HIGH_THRESHOLD (0)
#define PRESET_HIGH_CROSS_FILL (1)
#define PRESET_HIGH_LOOK_FAR (STEER_LOOK_FAR_DEFAULT)
#define PRESET_HIGH_DUTY (STRAIGHT_DUTY)
#define PRESET_HIGH_STOP_TIME (DRIVE_ARMED_TIMEOUT_S)

#endif /* CONFIG_H */
