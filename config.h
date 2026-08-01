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
#define OTSU_THRESHOLD_MIN (40)
#define OTSU_THRESHOLD_MAX (200)

// 前瞻相关参数
//
// 单行前瞻:菜单 Look Far 直接就是瞄准行(tr 坐标,越大越远;tr=1 是画面最下面一行)。
// 该行双边丢线时向近端滑到第一条有效行,实际用的行回报在 track_info_t.aim_row。
//
// 原先是"从 Look Far 起向近端取 20 行求平均",已删除:这 20 行跨越的地面距离
// 差了近一倍,像素/厘米标度不同,均匀平均等于隐含了一个偏向近端的加权。
// 换算:旧 Look Far N 的等效瞄准行 = N - 10(20 行窗口形心),
// 故 Low 76→66、Mid 79→69、默认 115→105,Kp/Kd 沿用不变。

#define STEER_LOOK_FAR_DEFAULT (105)
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
#define PRESET_LOW_KP (1.39f)
#define PRESET_LOW_KD (1.20f)
#define PRESET_LOW_D_ALPHA (0.40f)
#define PRESET_LOW_THRESHOLD (0)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_LOOK_FAR (66)
#define PRESET_LOW_DUTY (2600)
#define PRESET_LOW_STOP_TIME (20)

#define PRESET_MID_KP (1.55f)
#define PRESET_MID_KD (1.15f)
#define PRESET_MID_D_ALPHA (0.40f)
#define PRESET_MID_THRESHOLD (0)
#define PRESET_MID_CROSS_FILL (1)
#define PRESET_MID_LOOK_FAR (69)
#define PRESET_MID_DUTY (3000)
#define PRESET_MID_STOP_TIME (30)

#define PRESET_HIGH_KP (KP)
#define PRESET_HIGH_KD (KD)
#define PRESET_HIGH_D_ALPHA (D_FILT_ALPHA)
#define PRESET_HIGH_THRESHOLD (0)
#define PRESET_HIGH_CROSS_FILL (1)
#define PRESET_HIGH_LOOK_FAR (STEER_LOOK_FAR_DEFAULT)
#define PRESET_HIGH_DUTY (STRAIGHT_DUTY)
#define PRESET_HIGH_STOP_TIME (DRIVE_ARMED_TIMEOUT_S)

#endif /* CONFIG_H */
