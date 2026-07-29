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

/* 最长白列巡线:列扫描找自底向上连续白像素最长的列作为左右搜索基准 */
#define TH18_COL_MARGIN          (20)  /* 列扫描左右留白,避开图像边缘 */
#define TH18_CROSS_BOTH_LOST_MIN (10)  /* 十字检测:双边丢线行数下限 */
#define IMG_FILTER_SUM_MAX (255 * 5)   /* 3x3 去噪:邻域白点足够多则填白 */
#define IMG_FILTER_SUM_MIN (255 * 2)   /* 邻域白点过少则填黑 */

#define SERVO_PWM_HZ (50)
#define SERVO_CENTER (705)
#define SERVO_MIN (629)
#define SERVO_MAX (781)
#define SERVO_DIR (-1)

/* 分段 PD:kp = Min + (Max-Min)×(|e|/ESat)²,把"直道小误差"与"弯道大误差"
 * 解耦——直道靠 Min 稳、弯道靠 Max 转,不必用一个折中值同时应付两者。
 * 注意 Max/Min 比值不宜超过 3~4 倍、ESat 不宜过小,否则拐点过陡会退化成
 * "中心极软 + 突然满舵"的 bang-bang,重新引发饱和极限环 */
#define KP_MIN (1.09f)
#define KP_MAX (3.08f)
#define KP_E_SAT (13.0f)
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

/* 有效行数限速降级为纯安全网:只在视野真正劣化时兜底,不再与 Curve Duty
 * 抢弯速话语权——弯速由 Curve Duty 与曲率调度唯一决定,调多少就是多少。
 * 入弯口的减速已由 curve_temp 与 STRAIGHT_MIN_ROWS 覆盖,此表不再重复承担。
 * 各档上限均高于常用 Curve Duty,ROW 掉到 35 以下(视野严重劣化)才介入 */
#define ROWS_CAP_TABLE_LEN (3)
#define ROWS_CAP_BINS {20, 35, 55}
#define ROWS_CAP_DUTY {1500, 2200, 3400}

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

/* ---------------- Race Preset:低/中/高三档 ----------------
 * 菜单 Race Preset 进入子页面选择档位,ENTER 应用。
 * 应用时先 apply_defaults 全量恢复(含 Armed=OFF、Fine Step=OFF),
 * 再逐项覆盖——显式写全每一项,保证进入的是完整可复现的状态。 */

/* 低速档:实测可完赛的稳定组。Curve Duty == Str Duty == Duty == 2100,
 * 曲率削速与直道加速均不生效,全程恒速 —— 与"占空比全部设成相同值"的实测条件一致 */
#define PRESET_LOW_KP_MIN     (0.69f)
#define PRESET_LOW_KP_MAX     (1.77f)
#define PRESET_LOW_KP_E_SAT   (10.0f)
#define PRESET_LOW_KD         (4.65f)
#define PRESET_LOW_D_ALPHA    (0.40f)
#define PRESET_LOW_CURVE_DUTY (2100)
#define PRESET_LOW_STR_DUTY   (2100)
#define PRESET_LOW_THRESHOLD  (0)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_W_REF      (5700)
#define PRESET_LOW_DUTY       (2100)
#define PRESET_LOW_ST_JUDGE   (8)
#define PRESET_LOW_STOP_TIME  (105)

/* 中速档:实测可用组。
 * Curve Duty(2500) < Duty(3200) → 曲率削速生效,急弯降到 2500;
 * Str Duty(3200) == Duty → 直道确认不再额外提速。
 * 即"基准 3200、急弯 2500"的配置,与低速档的主要差别是基准速度和 W Ref */
#define PRESET_MID_KP_MIN     (0.79f)
#define PRESET_MID_KP_MAX     (1.70f)
#define PRESET_MID_KP_E_SAT   (13.0f)
#define PRESET_MID_KD         (4.73f)
#define PRESET_MID_D_ALPHA    (0.40f)
#define PRESET_MID_CURVE_DUTY (2500)
#define PRESET_MID_STR_DUTY   (3200)
#define PRESET_MID_THRESHOLD  (0)
#define PRESET_MID_CROSS_FILL (1)
#define PRESET_MID_W_REF      (6000)
#define PRESET_MID_DUTY       (3200)
#define PRESET_MID_ST_JUDGE   (8)
#define PRESET_MID_STOP_TIME  (28)

/* 高速档:占位,尚未实测。当前同为代码默认值 */
#define PRESET_HIGH_KP_MIN     (KP_MIN)
#define PRESET_HIGH_KP_MAX     (KP_MAX)
#define PRESET_HIGH_KP_E_SAT   (KP_E_SAT)
#define PRESET_HIGH_KD         (KD)
#define PRESET_HIGH_D_ALPHA    (D_FILT_ALPHA)
#define PRESET_HIGH_CURVE_DUTY (CURVE_DUTY)
#define PRESET_HIGH_STR_DUTY   (STRAIGHT_MAX_DUTY)
#define PRESET_HIGH_THRESHOLD  (0)
#define PRESET_HIGH_CROSS_FILL (1)
#define PRESET_HIGH_W_REF      (STEER_W_DUTY_REF)
#define PRESET_HIGH_DUTY       (STRAIGHT_DUTY)
#define PRESET_HIGH_ST_JUDGE   (STRAIGHT_JUDGE)
#define PRESET_HIGH_STOP_TIME  (DRIVE_ARMED_TIMEOUT_S)

#endif /* CONFIG_H */
