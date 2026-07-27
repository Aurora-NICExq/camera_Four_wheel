/* config.h */
#ifndef CONFIG_H
#define CONFIG_H

#define CAMERA_POWER_ON_DELAY_MS                                               \
  (500) /* 摄像头上电稳定延时，减轻初始化阶段画面撕裂 */

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
#define SERVO_CENTER (705)
#define SERVO_MIN (640)
#define SERVO_MAX (770)
#define SERVO_DIR (-1)

/* 丑牛 servoPID_ccd：P + P2*|e|*e + I + D*de；无舵机斜率限制 */
#define SERVO_P (2.5f)
#define SERVO_P2 (0.0f)
#define SERVO_I (0.0f)
#define SERVO_D (8.0f)
#define SERVO_A (0.0f) /* 陀螺仪角速度反馈；无 IMU 时保持 0 */

/* 丑牛 steering_type 速度：ex_speed + steer_up*(1-temp) + speed_up(直道) */
#define EX_SPEED_DUTY (2000)   /* car.ex_speed → 基础占空比 */
#define STEER_UP_DUTY (800)    /* car.steer_up  直道加成系数 */
#define SPEED_UP_DUTY (500)    /* car.speed_up  双直道确认后额外加成 */
#define CURVE_TEMP_DIV (30)    /* temp = |mid远- mid近| / 30，上限 1 */

/* 丑牛 straight() / speed_up_judge() */
#define STRAIGHT_JUDGE (8)     /* car.set_judge 边界共线判定阈值（像素） */
#define STRAIGHT_JUDGE_13 (25) /* car.judge_13 远近端差距过大则不算直道 */

#define MOTOR_TEST_DUTY (2000)
#define DUTY_HARD_CAP (6000)

#define DUTY_SLEW_DOWN (10000)
#define DUTY_SLEW_UP (120)

#define SLOW_MOTOR_STEP (50)   /* 丑牛缓启动：|ave-target|<=50 或已到目标后放开 */

#define FAILSAFE_MIN_ROWS (8)
#define FAILSAFE_MAX_BOTH_LOST_PCT (70)
#define FAILSAFE_SEVERE_BOTH_LOST_PCT (90)
#define FAILSAFE_FRAMES (10)
#define FAILSAFE_SEVERE_FRAMES (2)

#define MOTOR_PWM_FREQ (17000)

#define KEY_SCAN_PERIOD_MS                                                     \
  (5) /* 按键扫描周期；与实测参考工程 JOYSTICK_SCAN_PERIOD_MS 一致 */
#define KEY_DEBOUNCE_MS                                                        \
  (20) /* 消抖窗口：连续 KEY_DEBOUNCE_COUNT 次采样一致才翻转稳定态 */
#define KEY_DEBOUNCE_COUNT (KEY_DEBOUNCE_MS / KEY_SCAN_PERIOD_MS)
#define KEY_LONG_PRESS_MS (1000)
#define KEY_REPEAT_MS (80)

#endif /* CONFIG_H */
