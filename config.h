/* config.h */
#ifndef CONFIG_H
#define CONFIG_H

#define IMG_W (188)
#define IMG_H (120)
#define IMG_CENTER (94)
/* 曾有 FRAMES_PER_SECOND(50):全仓无人读取,纯孤儿宏,已删。
 * 帧率不参与任何计算——凡与挂钟时间相关的逻辑都用 hal_time_us(),
 * 因为主循环会因图像处理/屏幕刷新掉帧,按名义帧率折算时间是错的 */

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

/* 单套 PD:恒定比例增益,不随误差调度 */
#define KP (1.20f)
#define KD (1.49f)
#define D_FILT_ALPHA (0.4f)

#define STRAIGHT_DUTY (2100) /* 菜单 Duty 默认值:全程唯一的占空比(满量程 10000) */
#define DUTY_HARD_CAP (6000)



/* 减速不限幅:目标降低时立即跟进,见 control.c。
 * 曾有 DUTY_SLEW_DOWN(10000),但 g_duty_now ≤ DUTY_HARD_CAP(6000),
 * 单帧降幅恒 < 10000,该限幅分支永远走不到——已删,不要再加回来 */
#define DUTY_SLEW_UP (120)     /* 每帧最大升占空比；50fps 下 0→2000 约 0.8s */

/* 有效行数限速降级为纯安全网:只在视野真正劣化时兜底,不再与 Curve Duty
 * 抢速度话语权——速度由菜单 Duty 唯一决定。
 * ROW 掉到 20 以下(视野几乎丢失)才真正压到 Duty 之下 */
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

/* 无线 CSV 遥测:每 TELEM_DIV 帧发一行,约 50fps 下 10Hz */
#define TELEM_DIV (5u)
#define TELEM_UART_BAUD (115200u)
#define UART_TEST_PERIOD_US (200000u)

/* ---------------- Race Preset:低/中/高三档 ----------------
 * 菜单 Race Preset 进入子页面选择档位,ENTER 应用。
 * 应用时先 apply_defaults 全量恢复(含 Armed=OFF、Fine Step=OFF),
 * 再逐项覆盖——显式写全每一项,保证进入的是完整可复现的状态。 */

/* 低速档:实测验证可行组 Kp=2.29 Kd=1.49 Duty=2100 WRef=3800 */
#define PRESET_LOW_KP           (2.29f)
#define PRESET_LOW_KD         (1.49f)
#define PRESET_LOW_D_ALPHA    (0.40f)
#define PRESET_LOW_THRESHOLD  (0)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_W_REF      (3800)
#define PRESET_LOW_DUTY       (2100)
#define PRESET_LOW_STOP_TIME  (105)

/* 中速档:实测可用组 Kp=1.20 Kd=4.73 Duty=3200 WRef=6000。
 * 与低速档的差别只有基准速度和 W Ref(以及被迫抬高的 Kd)。
 * 注:Curve Duty / Str Duty 已于 c24f320 删除,速度全程只由菜单 Duty 决定,
 * 急弯降速只剩 rows_duty_cap 这一条图像质量安全网 */
#define PRESET_MID_KP           (KP)
#define PRESET_MID_KD         (4.73f)
#define PRESET_MID_D_ALPHA    (0.40f)
#define PRESET_MID_THRESHOLD  (0)
#define PRESET_MID_CROSS_FILL (1)
#define PRESET_MID_W_REF      (6000)
#define PRESET_MID_DUTY       (3200)
#define PRESET_MID_STOP_TIME  (28)

/* 高速档:占位,尚未实测。每一项都等于代码默认值,
 * 因此"应用高速档" == "Restore Def",Duty 仍是低速的 2100。
 * 菜单里已改名为 "High (= Default)",避免赛道上误以为选了一档快的。
 * 实测出真正的高速参数前,不要把名字改回 "High" */
#define PRESET_HIGH_KP           (KP)
#define PRESET_HIGH_KD         (KD)
#define PRESET_HIGH_D_ALPHA    (D_FILT_ALPHA)
#define PRESET_HIGH_THRESHOLD  (0)
#define PRESET_HIGH_CROSS_FILL (1)
#define PRESET_HIGH_W_REF      (STEER_W_DUTY_REF)
#define PRESET_HIGH_DUTY       (STRAIGHT_DUTY)
#define PRESET_HIGH_STOP_TIME  (DRIVE_ARMED_TIMEOUT_S)

#endif /* CONFIG_H */
