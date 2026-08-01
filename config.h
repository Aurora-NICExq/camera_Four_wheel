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

/* 巡线:最长白列 (Longest White Column)
   image_bin 行号 ir:0=画面顶(远),IMG_H-1=画面底(近)。
   自底向上逐列数连续白点 → 取最长的一列作为种子列 → 每行从种子列向左右扫边线。
   种子列在 [stop_row, LWC_START_ROW] 内必定为白(它就是那段连续白run),
   所以扫描起点永远落在赛道内,不需要参考实现里的"最长白列跳变保护"。 */

#define LWC_START_ROW (IMG_H - 2)
#define LWC_BORDER_MIN (2)         /* 左边界钳位值,同时也是左丢线时写入的值 */
#define LWC_BORDER_MAX (IMG_W - 3) /* 右边界钳位值,同时也是右丢线时写入的值 */
/* 种子列搜索范围:避开画面两侧,防止最长白列落在赛道外的反光/背景上 */
#define LWC_SCAN_COL_MIN (20)
#define LWC_SCAN_COL_MAX (IMG_W - 1 - 20)
#define LWC_FILTER_SUM_MAX (255 * 5)
#define LWC_FILTER_SUM_MIN (255 * 2)

/* 十字补线:拐点判据。
   上拐点 = 其上方若干行边线平滑、下方突然向外张开的那一行;下拐点反之。
   SMOOTH 是"平滑"的容差,JUMP 是"张开"的门限,二者不在同一条等值线上。
   张开方向带符号(左边线必须向左跳、右边线必须向右跳),所以"两侧同向变化
   = 弯道"这一类误判不需要额外阈值就被挡掉。 */
#define LWC_CROSS_SMOOTH (5)
#define LWC_CROSS_JUMP (15)
#define LWC_CROSS_EDGE_GUARD (4)  /* 拐点判据的探测跨度:比较 i 与 i±3、i±4 */
#define LWC_CROSS_SLOPE_SPAN (8)  /* 外推斜率的拟合基线长度(行) */

/* 弯道误触发拒绝器。
   BREAK_DROW 沿用八邻域版本的阈值;OPEN_WIDTH 同样沿用(188 宽的 ~74%),
   但统计区间已从"上拐点下方固定 10 行"改成**真实开口区间**
   (上拐点下一行 → 下拐点上一行),`LWC_CROSS_OPEN_ROW_MAX` 随之删除。 */
#define LWC_CROSS_BREAK_DROW (15)
#define LWC_CROSS_OPEN_WIDTH (140)
#define LWC_CROSS_OPEN_ROWS_MIN (4) /* 开口不足这么多行,不算十字 */

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
