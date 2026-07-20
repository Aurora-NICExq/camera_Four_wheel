/* config.h - tunable macros (no MCU headers); row=0 near car; duty 0..10000; time in frames */
#ifndef CONFIG_H
#define CONFIG_H

/* image / fps */
#define IMG_W                   (188)   /* 列数，须 = MT9V03X_W */
#define IMG_H                   (120)   /* 行数，须 = MT9V03X_H */
#define IMG_CENTER              (94)    /* IMG_W/2，转向零点 */
#define FRAMES_PER_SECOND       (50)    /* 帧时基；改帧率须重标 *_FRAMES */
#define COL_STEP                (1)     /* 列抽样；超时先改 2 再降帧率 */

/* 阈值：image_threshold==0 大津，>0 固定 */
#define FIXED_THRESHOLD         (128)   /* 大津兜底 / 手动参考 */
#define OTSU_ROW_STEP           (2)     /* 直方图行抽样 */
#define OTSU_COL_STEP           (2)     /* 直方图列抽样 */
#define OTSU_THRESHOLD_MIN      (40)    /* 大津下限保护 */
#define OTSU_THRESHOLD_MAX      (200)   /* 大津上限保护 */

/* edges / mid */
#define WIDTH_MIN_PX            (20)    /* 合法宽度下限 */
#define WIDTH_MAX_PX            (186)   /* 合法宽度上限 */
#define WIDTH_TABLE_DEFAULT     (110)   /* 无学习时的兜底宽度 */

/* 转向误差 = 各行 (mid-CENTER) 加权；带 0 最近 */
#define STEER_W_BANDS           (8)
#define STEER_W_BAND_ROWS       (15)
#define STEER_WEIGHTS_LOWSPEED  { 8, 10, 9, 6, 4, 2, 1, 0 }
#define STEER_WEIGHTS_HIGHSPEED { 2,  4, 6, 9, 10, 8, 5, 2 }
#define STEER_W_SINGLE_EDGE_PCT (50)    /* 单边重建行权重% */
#define STEER_W_BOTH_LOST_PCT   (0)     /* 普通双边丢失权重% */
#define STEER_W_CROSS_FILL_PCT  (70)    /* 十字补线行权重% */

/* 曲率 = 远段斜率 − 近段，Q8(×256) */
#define CURV_NEAR_ROW_LO        (5)
#define CURV_NEAR_ROW_HI        (25)
#define CURV_FAR_ROW_LO         (45)
#define CURV_FAR_ROW_HI         (70)
#define CURV_MIN_SPAN_ROWS      (6)

/* detectors */
#define ENABLE_CROSS            (1)
#define ENABLE_RING             (1)
#define ENABLE_RAMP             (0)

#define CROSS_BAND_ROW_LO       (10)
#define CROSS_BAND_ROW_HI       (60)
#define CROSS_MIN_BOTH_LOST     (12)
#define CROSS_MIN_GOOD_BELOW    (5)
#define CROSS_MIN_GOOD_ABOVE    (5)
#define CROSS_EXIT_MAX_LOST     (4)
#define CROSS_EXIT_CONFIRM      (3)

/* 18th 最长白列 / 十字（移植 the-18th-smartcar） */
#define TH18_COL_MARGIN         (20)    /* 最长白列搜索左右留白 */
#define TH18_CROSS_BOTH_LOST_MIN (10)   /* 十字：双边丢线行数下限 */

#define RING_BAND_ROW_LO        (8)
#define RING_BAND_ROW_HI        (70)
#define RING_ARC_MIN_LOST       (8)
#define RING_ARC_MIN_GOOD_ABOVE (4)
#define RING_SOLID_MAX_LOST     (5)
#define RING_ARC_MIN_GOOD_BELOW (4)
#define RING_ENTRY_ROW          (18)
#define RING_ENTRY_CONFIRM      (3)
#define RING_IN_MIN_FRAMES      (15)
#define RING_EXIT_BREAK_LOST    (10)
#define RING_EXIT_BREAK_CONFIRM (3)
#define RING_EXIT_DONE_MAX_LOST (3)
#define RING_EXIT_CONFIRM       (4)
#define RING_EXIT_BIAS_PX       (25)

#define RAMP_BAND_ROW_LO        (50)
#define RAMP_BAND_ROW_HI        (75)
#define RAMP_WIDTH_NUM          (140)
#define RAMP_MIN_ROWS           (10)
#define RAMP_HOLD_FRAMES        (40)

/* fsm */
#define CROSS_CONFIRM_M         (4)
#define CROSS_CONFIRM_N         (6)
#define RING_CONFIRM_M          (4)
#define RING_CONFIRM_N          (6)
#define RAMP_CONFIRM_M          (4)
#define RAMP_CONFIRM_N          (6)

#define CROSS_TIMEOUT_FRAMES    (60)
#define RING_PRE_TIMEOUT_FRAMES (75)
#define RING_IN_TIMEOUT_FRAMES  (150)
#define RING_EXIT_TIMEOUT_FRAMES (60)
#define RAMP_TIMEOUT_FRAMES     (80)

#define RECOVERY_CONFIRM_FRAMES (8)
#define RECOVERY_TIMEOUT_FRAMES (40)
#define RECOVERY_MIN_ROWS       (45)
#define RECOVERY_MAX_BOTH_LOST  (3)

#define COOLDOWN_FRAMES         (25)
#define COOLDOWN_MASK           (DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP)

#define CROSS_DUTY_CAP          (4000)
#define RING_PRE_DUTY_CAP       (2800)
#define RING_IN_DUTY_CAP        (3000)
#define RING_EXIT_DUTY_CAP      (3000)
#define RAMP_DUTY_CAP           (3200)
#define RECOVERY_DUTY_CAP       (2600)

/* steer：50Hz→1 duty=2µs；MIN/MAX 为机械极限，超程伤齿轮 */
#define SERVO_PWM_HZ            (50)
#define SERVO_CENTER            (770)   /* 直行中位 duty */
#define SERVO_MIN               (685)   /* 左极限 */
#define SERVO_MAX               (850)   /* 右极限 */
#define SERVO_DIR               (+1)    /* +1=正误差打右；反了改 -1 */
#define SERVO_SLEW_LIMIT        (45)    /* 每帧最大舵机步进 */

/* PD：Kp 随 |e| 二次调度 */
#define USE_CONST_KP            (0)     /* 1=恒定 Kp 调试 */
#define KP_CONST                (1.8f)
#define KP_MIN                  (1.0f)  /* |e|=0 */
#define KP_MAX                  (3.2f)  /* |e|≥KP_E_SAT */
#define KP_E_SAT                (40.0f) /* 像素 */
#define KD                      (6.0f)
#define D_FILT_ALPHA            (0.4f)  /* D 项 EMA，抑量化毛刺 */

/* speed：基准 duty + 硬上限 + 斜坡 */
#define STRAIGHT_DUTY           (4500)
#define DUTY_HARD_CAP           (6000)  /* 满量程 10000 的 60% */
#define DUTY_SLEW_DOWN          (10000) /* 每帧最大降（默认不限） */
#define DUTY_SLEW_UP            (120)   /* 每帧最大升；兼软启动 */
#define ENABLE_HW_BRAKE         (0)     /* 未接线 */
#define ENABLE_VBAT_COMP        (0)     /* 无分压 ADC */

/* safety */
#define FAILSAFE_MIN_ROWS       (8)     /* 有效行过少=看不见赛道 */
#define FAILSAFE_MAX_BOTH_LOST_PCT (70) /* 双边丢失%→图像失效 */
#define FAILSAFE_SEVERE_BOTH_LOST_PCT (90) /* 严重全白阈值 */
#define FAILSAFE_FRAMES         (10)    /* 一般失效确认帧 */
#define FAILSAFE_SEVERE_FRAMES  (2)     /* 严重全白快速断油 */
#define CAMERA_WATCHDOG_US      (100000u) /* 武装后 100ms 无帧→断油 */
#define STARTUP_DELAY_MS        (2000)  /* 上电等待（相机前唯一 ms） */
#define STARTUP_BEEP_MS         (100)
#define DEBUG_NO_DRIVE          (1)     /* 1=流水线跑、duty 强制 0 */

/* TEST_COAST 滑行标定 */
#define TEST_COAST              (0)
#define TEST_COAST_CRUISE_DUTY  (4000)
#define TEST_COAST_TARGET_DUTY  (0)
#define TEST_COAST_MARKER_ROWS  (30)    /* 有效行骤降=黑标记条 */

/* display / motor */
#define MOTOR_PWM_HZ            (17000) /* 与舵机须不同 ATOM */
#define MOTOR_DIR_FORWARD_LEVEL (0)     /* 0=低电平正转 */
#define DISPLAY_TEXT_DIV        (25)    /* 文本刷新分频 */
#define DISPLAY_IMG_DIV         (10)
#define DISPLAY_IMG_W           (94)    /* 188/2 降采样 */
#define DISPLAY_IMG_H           (60)
#if (DISPLAY_IMG_W > 240) || (DISPLAY_IMG_H > 320)
#error "IPS200 竖屏 240x320：DISPLAY_IMG_W/H 不得超过 240/320"
#endif
#define ENABLE_UART_TELEMETRY   (1)
#define TELEMETRY_DIV           (25)
#define ENABLE_UART_IMAGE       (0)     /* 整帧串口图，仅静态调试 */

#define CHIRP_FRAMES_SHORT      (3)     /* 十字 */
#define CHIRP_FRAMES_MID        (8)     /* 环岛/坡道/恢复 */
#define CHIRP_FRAMES_LONG       (20)    /* 故障/失控 */

/* KEY_IDX_COAST 仅 TEST_COAST=1；常规键由菜单占用 */
#define KEY_IDX_COAST           (2)

#endif /* CONFIG_H */
