/* config.h */
#ifndef CONFIG_H
#define CONFIG_H

#define IMG_W (188)
#define IMG_H (120)
#define IMG_CENTER (94)
#define FRAMES_PER_SECOND (50)

/* arm 后延时发车 + 定时停车,基于 STM 硬件时钟(hal_time_us)按挂钟时间计:
 * 帧计数会因图像处理/屏幕刷新拖慢主循环而掉帧合并、严重失真,挂钟时间不会。
 * 计时起点 = arm 后第一个有效帧;2s 内整车静止(舵机保持回中),之后软启动发车;
 * 发车后跑满菜单 Stop Time 秒锁存停车,Armed 行显示 TMO,重新 OFF→ON 复位 */
#define DRIVE_LAUNCH_DELAY_S (2)
#define DRIVE_ARMED_TIMEOUT_S (30) /* 菜单 Stop Time 默认值(秒) */
#define DRIVE_LAUNCH_DELAY_US ((uint32_t)DRIVE_LAUNCH_DELAY_S * 1000000u)
/* 计时不依赖时间戳单调:逐帧累计 dt,单帧差值异常(定时器回绕/毛刺)
 * 时按名义帧周期累计,避免回绕导致的瞬间误超时 */
#define DRIVE_DT_CLAMP_US (200000u)
#define DRIVE_DT_NOMINAL_US (20000u)

#define FIXED_THRESHOLD (128)
#define OTSU_ROW_STEP (2)
#define OTSU_COL_STEP (2)
#define OTSU_THRESHOLD_MIN (40)
#define OTSU_THRESHOLD_MAX (200)

#define STEER_W_BANDS (8)
#define STEER_W_BAND_ROWS (15)
#define STEER_WEIGHTS_LOWSPEED {8, 10, 9, 6, 4, 2, 1, 0}
#define STEER_WEIGHTS_HIGHSPEED {2, 4, 6, 9, 10, 8, 5, 2}
/* 前瞻归一化基准:占空比达到该值时权重完全切到高速(远端)表。
 * 必须设成实际能跑到的顶速,设成 DUTY_HARD_CAP 会让远端表永远吃不满,
 * 车始终"看半近半远"、等偏差出现才起手,峰值打角偏大 */
#define STEER_W_DUTY_REF (3800)
#define STEER_W_SINGLE_EDGE_PCT (50)
#define STEER_W_CROSS_FILL_PCT (70) /* 十字补线行降权:补出的边界是外推值,不如实测边线可信 */
/* 盲区误差保持:双边丢线行没有中线信息,不再投"假居中"票;
 * 有效权重塌陷(视野基本全是开口)时沿用进入盲区前的误差,
 * 最多保持 N 帧,超时每帧 ×3/4 衰减回中 */
#define ERR_HOLD_W_MIN (120)
#define ERR_HOLD_MAX_FRAMES (20)

#define EIGHTN_START_ROW (IMG_H - 2)
#define EIGHTN_BORDER_MIN (1)
#define EIGHTN_BORDER_MAX (IMG_W - 2)
#define EIGHTN_MAX_POINTS (IMG_H * 3)
#define EIGHTN_FILTER_SUM_MAX (255 * 5)
#define EIGHTN_FILTER_SUM_MIN (255 * 2)
#define EIGHTN_MEET_DIST (2)
#define EIGHTN_EDGE_LOST_MARGIN  (2)          /* 边界贴到图像黑框(±2px)视作丢线:沿框爬行的链条不含中线信息 */

/* 十字补线:菜单 Cross Fill 开关控制,便于开/关背靠背对比 */
#define EIGHTN_CROSS_SLOPE_BACK (15)
#define EIGHTN_CROSS_SLOPE_NEAR  (5)
#define EIGHTN_CROSS_CORNER_L    (4)
#define EIGHTN_CROSS_CORNER_R    (IMG_W - 4)
#define EIGHTN_CROSS_BREAK_DROW  (15)         /* 十字左右上拐点行号最大允许偏差 */
#define EIGHTN_CROSS_OPEN_WIDTH  (140)        /* 十字开口判定最小宽度(像素) */
#define EIGHTN_CROSS_OPEN_ROW_MAX (IMG_H - 8) /* 开口采样最近行:再近的行正常赛道也接近全宽,不作依据 */

#define SERVO_PWM_HZ (50)
#define SERVO_CENTER (705)
#define SERVO_MIN (629)
#define SERVO_MAX (781)
#define SERVO_DIR (-1)

/* 单套 PD:恒定比例增益,不再随误差调度。
 * 满舵门槛 = 舵机半行程/Kp = 76/3.08 ≈ 25px,弯道大误差照常满舵。
 * 调参只剩两个旋钮:Kp 管"转得够不够",Kd 管"这个速度配不配转" */
#define KP (3.08f)
#define KD (1.49f)
#define D_FILT_ALPHA (0.4f)

#define STRAIGHT_DUTY (2900) /* 基准占空比(菜单 Duty 默认值,满量程 10000) */
#define DUTY_HARD_CAP (6000)

/* 曲率速度调度:急弯目标为绝对值 Curve Duty,不随 Duty 上浮:
 * target = Duty - (Duty - CurveDuty)×temp;直道确认(且行数足够)只增不减 = Str Duty */
#define CURVE_DUTY (2400)        /* 急弯(temp=1)绝对目标占空比 */
#define STRAIGHT_MAX_DUTY (3700) /* 直道确认目标占空比 */
#define STRAIGHT_MIN_ROWS (80)   /* 直道确认所需最少有效行,视野变短即提前退出加速 */

/* 出弯确认门(master exit gate 精简版):出弯后误差连续收敛 N 帧
 * 才放行直道加速,出口瞬态未消化时不让增益与速度同时上涨点燃直道摇摆 */
#define EXIT_ERR_MAX (10)
#define EXIT_ROWS_MIN (70)
#define EXIT_MAX_BOTH_LOST (6)
#define EXIT_CONFIRM_FRAMES (6)
#define CURVE_TEMP_DIV (30)    /* 曲率归一化除数，temp 上限 1 */
#define STRAIGHT_JUDGE (8)     /* 边界共线判定阈值（像素） */
#define STRAIGHT_JUDGE_13 (25) /* 远近端差距过大则不算直道 */
#define SLOW_MOTOR_STEP (50)   /* 缓启动：|target-now|<=50 或已到目标后放开 */

#define DUTY_SLEW_DOWN (10000) /* 减速不限幅，目标降低时立即跟进 */
#define DUTY_SLEW_UP (120)     /* 每帧最大升占空比；50fps 下 0→2000 约 0.8s */

/* 有效行数限速(master cap_rows 通道):入弯口视野塌缩早于近端误差出现,
 * 按可见行数封顶目标占空比;直道确认加速同样受此上限约束 */
#define ROWS_CAP_TABLE_LEN (5)
#define ROWS_CAP_BINS {25, 45, 65, 85, 105}
#define ROWS_CAP_DUTY {1800, 2200, 2600, 3200, DUTY_HARD_CAP}

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

/* Race Preset:实测验证可行的一组参数,菜单一键切换
 * (先恢复全部默认,再覆盖以下四项,保证进入的是完整的已验证状态) */
#define PRESET_KP         (3.08f)
#define PRESET_CURVE_DUTY (2400)
#define PRESET_STR_DUTY   (3700)
#define PRESET_DUTY       (2900)

#endif /* CONFIG_H */
