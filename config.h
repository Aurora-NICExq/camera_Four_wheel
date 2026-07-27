/* config.h */
#ifndef CONFIG_H
#define CONFIG_H

#define IMG_W (188)
#define IMG_H (120)
#define IMG_CENTER (94)
#define FRAMES_PER_SECOND (50)

#define FIXED_THRESHOLD (128)
#define OTSU_ROW_STEP (2)
#define OTSU_COL_STEP (2)
#define OTSU_THRESHOLD_MIN (40)
#define OTSU_THRESHOLD_MAX (200)

#define STEER_W_BANDS (8)
#define STEER_W_BAND_ROWS (15)
#define STEER_WEIGHTS_LOWSPEED {8, 10, 9, 6, 4, 2, 1, 0}
#define STEER_WEIGHTS_HIGHSPEED {2, 4, 6, 9, 10, 8, 5, 2}
#define STEER_W_SINGLE_EDGE_PCT (50)
#define STEER_W_BOTH_LOST_PCT (40)
#define STEER_W_CROSS_FILL_PCT (70)

#define EIGHTN_START_ROW (IMG_H - 2)
#define EIGHTN_BORDER_MIN (1)
#define EIGHTN_BORDER_MAX (IMG_W - 2)
#define EIGHTN_MAX_POINTS (IMG_H * 3)
#define EIGHTN_FILTER_SUM_MAX (255 * 5)
#define EIGHTN_FILTER_SUM_MIN (255 * 2)
#define EIGHTN_MEET_DIST (2)
#define EIGHTN_CROSS_SLOPE_BACK (15)
#define EIGHTN_CROSS_SLOPE_NEAR  (5)
#define EIGHTN_CROSS_CORNER_L    (4)
#define EIGHTN_CROSS_CORNER_R    (IMG_W - 4)

#define SERVO_PWM_HZ (50)
#define SERVO_CENTER (705)
#define SERVO_MIN (629)
#define SERVO_MAX (781)
#define SERVO_DIR (-1)
#define SERVO_SLEW_LIMIT (45)

#define KP_MIN (1.09f)
#define KP_MAX (9.48f)
#define KP_E_SAT (35.0f)
#define KD (1.49f)
#define D_FILT_ALPHA (0.4f)

#define STRAIGHT_DUTY (2500) /* 基准占空比 25%（满量程 10000） */
#define DUTY_HARD_CAP (6000)

/* 曲率速度调度：target = Duty - CurveCut×temp；直道确认后 = Str Duty
 * 默认：弯底 20% / 基准 25% / 直道 32%（菜单可调） */
#define CURVE_CUT_DUTY (500)   /* 急弯(temp=1)从基准削减量 → 25%-5%=20% */
#define STRAIGHT_MAX_DUTY (3200) /* 直道确认目标占空比 32% */
#define CURVE_TEMP_DIV (30)    /* 曲率归一化除数，temp 上限 1 */
#define STRAIGHT_JUDGE (8)     /* 边界共线判定阈值（像素） */
#define STRAIGHT_JUDGE_13 (25) /* 远近端差距过大则不算直道 */
#define SLOW_MOTOR_STEP (50)   /* 缓启动：|target-now|<=50 或已到目标后放开 */

#define DUTY_SLEW_DOWN (10000) /* 减速不限幅，目标降低时立即跟进 */
#define DUTY_SLEW_UP (120)     /* 每帧最大升占空比；50fps 下 0→2000 约 0.8s */

#define FAILSAFE_MIN_ROWS (8)
#define FAILSAFE_MAX_BOTH_LOST_PCT (70)
#define FAILSAFE_SEVERE_BOTH_LOST_PCT (90)
#define FAILSAFE_FRAMES (10)
#define FAILSAFE_SEVERE_FRAMES (2)

#define MOTOR_PWM_FREQ (17000)

#define KEY_SCAN_PERIOD_MS (5)  /* 按键扫描周期；与实测参考工程 JOYSTICK_SCAN_PERIOD_MS 一致 */
#define KEY_DEBOUNCE_MS (20)    /* 消抖窗口：连续 KEY_DEBOUNCE_COUNT 次采样一致才翻转稳定态 */
#define KEY_DEBOUNCE_COUNT (KEY_DEBOUNCE_MS / KEY_SCAN_PERIOD_MS)
#define KEY_LONG_PRESS_MS (1000) /* UP/DOWN 按住超过该时长后开始连发 */
#define KEY_REPEAT_MS (80)       /* 连发间隔（is_repeat=1，菜单按 10 倍步长调整） */

#define MOTOR_TEST_DUTY (2000) /* 电机测试固定占空比 20% */

#endif /* CONFIG_H */
