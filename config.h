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

#define CURV_NEAR_ROW_LO (5)
#define CURV_NEAR_ROW_HI (25)
#define CURV_FAR_ROW_LO (45)
#define CURV_FAR_ROW_HI (70)
#define CURV_MIN_SPAN_ROWS (6)

#define EIGHTN_START_ROW (IMG_H - 2)
#define EIGHTN_BORDER_MIN (1)
#define EIGHTN_BORDER_MAX (IMG_W - 2)
#define EIGHTN_MAX_POINTS (IMG_H * 3)
#define EIGHTN_FILTER_SUM_MAX (255 * 5)
#define EIGHTN_FILTER_SUM_MIN (255 * 2)
#define EIGHTN_CROSS_SLOPE_BACK (15)
#define EIGHTN_CROSS_SLOPE_NEAR (5)
#define EIGHTN_CROSS_CORNER_L (4)
#define EIGHTN_CROSS_CORNER_R (IMG_W - 4)
#define EIGHTN_MEET_DIST (2)

#define SERVO_PWM_HZ (50)
#define SERVO_CENTER (770)
#define SERVO_MIN (685)
#define SERVO_MAX (850)
#define SERVO_DIR (-1)
#define SERVO_SLEW_LIMIT (45)

#define KP_MIN (1.0f)
#define KP_MAX (3.2f)
#define KP_E_SAT (40.0f)
#define KD (6.0f)
#define D_FILT_ALPHA (0.4f)

#define STRAIGHT_DUTY (2000) /* 直道 20%（满量程 10000） */
#define DUTY_HARD_CAP (6000)
#define MIN_TURN_DUTY (2600)

#define ROWS_DUTY_TABLE_LEN (5)
#define ROWS_DUTY_TABLE_ROWS {25, 45, 65, 85, 105}
#define ROWS_DUTY_TABLE_DUTY {2600, 3200, 3800, 4200, 6000}

#define CURV_DUTY_TABLE_LEN (4)
#define CURV_DUTY_TABLE_CURV {40, 90, 160, 260}
#define CURV_DUTY_TABLE_DUTY {6000, 3800, 3200, 2600}

#define STEER_DUTY_SLOPE_NUM (18)
#define STEER_DUTY_SLOPE_DEN (1)

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

#endif /* CONFIG_H */
