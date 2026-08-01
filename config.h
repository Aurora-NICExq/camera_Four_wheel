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

//前瞻相关参数

#define STEER_LOOK_SPAN (20)
#define STEER_LOOK_FAR_DEFAULT (115)
#define STEER_LOOK_FAR_MAX                                                     \
  (IMG_H - 1)
#if (STEER_LOOK_SPAN < 1) || (STEER_LOOK_SPAN >= STEER_LOOK_FAR_MAX)
#error "STEER_LOOK_SPAN out of range"
#endif
#if (STEER_LOOK_FAR_DEFAULT > STEER_LOOK_FAR_MAX) ||                           \
    (STEER_LOOK_FAR_DEFAULT <= STEER_LOOK_SPAN)
#error "STEER_LOOK_FAR_DEFAULT out of range"
#endif
#define ERR_HOLD_MAX_FRAMES (20)

/* 巡线:最长白列 — 完整移植参考库 Center_line_deal_plus(23,163)。
   行号 ir:0=画面顶(远),IMG_H-1=画面底(近)。 */

#define LWC_SCAN_START_ROW (IMG_H - 1) /* 参考库 IMAGE_HEIGHT-1,边线扫描起点 */
#define LWC_SCAN_COL_MIN (23)
#define LWC_SCAN_COL_MAX (163)
#define LWC_COL_JUMP_MAX (60)          /* 参考库种子列跳变保护阈值 */
#define LWC_FILTER_SUM_MAX (255 * 5)
#define LWC_FILTER_SUM_MIN (255 * 2)

/* 十字补线:整段按参考库 image.c 原样移植(Find_Up_Point / Find_Down_Point /
   Cross_Detect / Left_Add_Line / Lengthen_*)。判据里的 5 / 8 / 15 / 30 / 10
   等阈值是参考库写死的字面量,移植时保持原样写在 image.c 里,
   **没有提成宏** —— 提成宏会造成"这些数是可调旋钮"的错觉,而它们
   在参考库里从来没有被单独标定过(R2)。要调请直接改 image.c 并做实测。 */

//舵机参数

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

//保存的三版参数

// 这个他妈测一次就出来了，前瞻是对的
#define PRESET_LOW_KP (1.50f)
#define PRESET_LOW_KD (1.20f)
#define PRESET_LOW_D_ALPHA (0.40f)
#define PRESET_LOW_THRESHOLD (0)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_LOOK_FAR (71)
#define PRESET_LOW_DUTY (2700)
#define PRESET_LOW_STOP_TIME (30)

#define PRESET_MID_KP (1.55f)
#define PRESET_MID_KD (1.15f)
#define PRESET_MID_D_ALPHA (0.40f)
#define PRESET_MID_THRESHOLD (0)
#define PRESET_MID_CROSS_FILL (1)
#define PRESET_MID_LOOK_FAR (79)
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
