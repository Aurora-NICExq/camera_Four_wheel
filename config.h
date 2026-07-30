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

/* ---------------- 两段前瞻 ----------------
 * 近段 [0, STEER_SPLIT_ROW) 与远段 [STEER_SPLIT_ROW, STEER_FAR_ROW_HI)
 * 各自均匀平均,再按菜单 Far W% 混合成一个误差标量。
 * 段内均匀 = 没有隐藏的权重曲线;公式里不含 duty = 转向增益不随速度漂移。
 *
 * 分段点取值依据。被替换掉的 8 段表,在两组实测预设的混合系数(k≈137)下
 * 塌缩成这条固定权重曲线(按峰值归一):
 *     段:   0     1     2     3     4     5     6     7
 *     行:  0-14 15-29 30-44 45-59 60-74 75-89 90-104 105-119
 *     权:  0.63  0.89  0.97  1.00  0.95  0.68  0.41  0.14
 * 峰在第 3 段(45-59 行)→ 取 45 作分段点。
 * 90 行以上原权重只剩 0.41/0.14,又是透视压缩最重、最先丢线的区域
 * → 取 90 作远段上界,105-119 整段弃用。
 *
 * Far W% 默认 65 的依据:近段行心≈22、远段行心≈67,
 * 混合行心 = (22*(100-x) + 67*x)/100,x=65 时 ≈51.2,
 * 与旧 8 段表满视野下的等效瞄准行(~51)对齐 → Kp/Kd 可基本沿用。
 *
 * 已删除的旧机制:STEER_W_BANDS / BAND_ROWS / WEIGHTS_LOWSPEED /
 * WEIGHTS_HIGHSPEED / STEER_W_DUTY_REF(菜单 W Ref)/ CROSS_FILL_PCT。
 * 速度交叉淡入从未真正生效:两组实测预设算出的 k 分别是 141 和 136,
 * W Ref 一直被用来手动抵消速度依赖,"高速看远"这个特性一次没被用上。 */
#define STEER_SPLIT_ROW  (45)
#define STEER_FAR_ROW_HI (90)
#define STEER_FAR_W_PCT  (65) /* 菜单 Far W% 默认值:远段占比,近段 = 100-该值 */

/* 单边行 mid 重建用的半宽兜底值。仅在"整个搜索带里没有任何一条双边可见行"
 * 时才用得到——那种帧本身已经严重劣化,丢线保护大概率正在计数。
 * 正常帧的半宽由 export_track 从最近的双边可见行滚动取得,不用这个值。
 * 曾有 STEER_W_SINGLE_EDGE_PCT(50) 给假 mid 打折,已随上游修好而删除 */
#define TRACK_HALF_W_FALLBACK (60)

/* 盲区误差保持:双边丢线行没有中线信息,不投"假居中"票。
 * 近段远段都没有任何一行投票时,沿用进入盲区前的误差,最多保持 N 帧,
 * 超时每帧 ×3/4 衰减回中。err_hold 送遥测,陈旧误差不再伪装成实测值。
 * 曾以 ERR_HOLD_W_MIN(120) 作触发阈值,但那是"混合权重之和"的单位——
 * 改动权重表就会悄悄改变它的含义(R1),已换成"投票行数为 0"这个定死的条件 */
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

/* 曾有 ROWS_CAP 行数限速表({20,35,55} → {1500,2200,3400}),已整表删除。
 * 删除理由:低速档 Duty=2100 时 3400/2200 两档恒大于 Duty、永不生效,
 * 唯一活跃行为是"ROW<20 压到 1500";而 ROW<8 已由下面的丢线保护接管,
 * 独占区间只剩 ROW∈[8,19]。另一条理由(当时成立,现已不适用):它压低 duty
 * 会经 control_duty_prev 反馈进 image_process 改变权重表混合系数 k,
 * 构成隐式增益调度——该耦合已随两段前瞻改造整体消失,control_duty_prev 也已删除。
 * 现在图像劣化时的唯一保护 = 丢线保护(连续 10 帧 / severe 2 帧后停车),
 * 速度全程只由菜单 Duty 决定。 */

/* 丢线保护:图像劣化时的唯一兜底,二值杀开关——不减速,直接停车 */
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

/* 菜单 UART Test 页发送周期。按挂钟时间计(hal_time_us),不用帧计数——
 * 主循环会因图像处理/屏幕刷新掉帧,帧计数出来的"5Hz"是不准的。
 * 5Hz 足够人在 PC 助手里看清序号连续性,又不会把队列打满 */
#define UART_TEST_PERIOD_US (200000u)

/* 无线 CSV 遥测:每 TELEM_DIV 帧发一行,约 50fps 下 10Hz */
#define TELEM_DIV (5u)
/* 无线模块波特率:115200 对当前 CSV 遥测(约10Hz)足够。须与 PC 助手、
 * 逐飞库 WIRELESS_UART_BUAD_RATE 一致,见 seekfree_baud.h */
#define TELEM_UART_BAUD (115200u)

/* ---------------- Race Preset:低/中/高三档 ----------------
 * 菜单 Race Preset 进入子页面选择档位,ENTER 应用。
 * 应用时先 apply_defaults 全量恢复(含 Armed=OFF、Fine Step=OFF),
 * 再逐项覆盖——显式写全每一项,保证进入的是完整可复现的状态。 */

/* 低速档:实测验证可行组 Kp=2.29 Kd=1.49 Duty=2100。
 * 原记录含 WRef=3800,该参数已随速度交叉淡入一并删除;
 * Far W% 沿用默认 65(混合行心与旧 WRef=3800 下的等效瞄准行对齐) */
#define PRESET_LOW_KP           (2.29f)
#define PRESET_LOW_KD         (1.49f)
#define PRESET_LOW_D_ALPHA    (0.40f)
#define PRESET_LOW_THRESHOLD  (0)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_FAR_W      (STEER_FAR_W_PCT)
#define PRESET_LOW_DUTY       (2100)
#define PRESET_LOW_STOP_TIME  (105)

/* 中速档:实测可用组 Kp=1.20 Kd=4.73 Duty=3200。
 * 与低速档的差别只有基准速度(原记录还有 WRef=6000,该参数已删)。
 * Kd 被迫抬到 4.73 的原因见 CLAUDE.md 证据 2:旧权重表把瞄准点向近端拖,
 * 只能靠 Kd 微分回来。两段前瞻上车后 Kd 大概率能压回 1.5 附近,重新实测。
 * 注:Curve Duty / Str Duty 已于 c24f320 删除,ROWS_CAP 行数限速表随后也已删除,
 * 速度全程只由菜单 Duty 决定,不存在任何自动降速 */
#define PRESET_MID_KP           (KP)
#define PRESET_MID_KD         (4.73f)
#define PRESET_MID_D_ALPHA    (0.40f)
#define PRESET_MID_THRESHOLD  (0)
#define PRESET_MID_CROSS_FILL (1)
#define PRESET_MID_FAR_W      (STEER_FAR_W_PCT)
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
#define PRESET_HIGH_FAR_W      (STEER_FAR_W_PCT)
#define PRESET_HIGH_DUTY       (STRAIGHT_DUTY)
#define PRESET_HIGH_STOP_TIME  (DRIVE_ARMED_TIMEOUT_S)

#endif /* CONFIG_H */
