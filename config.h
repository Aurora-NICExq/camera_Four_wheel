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

/* 单段前瞻:从 Look Far(菜单)这一行往近端滑,取 STEER_LOOK_SPAN 个
 * "至少有一侧边线"的行做均匀平均。全部权重在这一段上。
 *
 * Look Far 调大 = 从更远起搜(更快、更提前打角);调小 = 窗口整体移近。
 * 预期:低速/急弯赛道 85~100,高速直道为主 105~115;默认 115。
 * span 写死 20——调"收多少行"会和 Far 互抵(R1),不开菜单。
 *
 * 看不到远处就继续往近滑,前瞻只会变短不会整窗落空(见 look_ahead_error)。 */
#define STEER_LOOK_SPAN        (20)
#define STEER_LOOK_FAR_DEFAULT (115)
#define STEER_LOOK_FAR_MAX     (IMG_H - 1) /* tr = 1 + (r-1) 时 r 最大 Far-1 < IMG_H */
#if (STEER_LOOK_SPAN < 1) || (STEER_LOOK_SPAN >= STEER_LOOK_FAR_MAX)
#error "STEER_LOOK_SPAN out of range"
#endif
#if (STEER_LOOK_FAR_DEFAULT > STEER_LOOK_FAR_MAX) || (STEER_LOOK_FAR_DEFAULT <= STEER_LOOK_SPAN)
#error "STEER_LOOK_FAR_DEFAULT out of range"
#endif
/* 曾有 8 段低/高速权重表 + W Ref 按 duty 交叉淡入,已删。
 * 曾有 STEER_LOOK_LO/HI 成对宏:LO 在滑窗语义下只是 span,易误调,已拆开。
 * 曾有单边/补线逐行折扣,已删。 */
/* 盲区误差保持:前瞻窗内无有效行时沿用进入盲区前的误差,
 * 最多保持 N 帧,超时每帧 ×3/4 衰减回中 */
#define ERR_HOLD_MAX_FRAMES (20)

/* 八邻域双边巡线(已从最长白列硬回退) */
#define EIGHTN_START_ROW (IMG_H - 2)
#define EIGHTN_BORDER_MIN (1)
#define EIGHTN_BORDER_MAX (IMG_W - 2)
#define EIGHTN_MAX_POINTS (IMG_H * 3)
#define EIGHTN_FILTER_SUM_MAX (255 * 5)
#define EIGHTN_FILTER_SUM_MIN (255 * 2)
#define EIGHTN_MEET_DIST (2)
#define EIGHTN_EDGE_LOST_MARGIN  (2)          /* 边界贴到图像黑框(±2px)视作丢线 */

/* 十字补线:菜单 Cross Fill 开关控制 */
#define EIGHTN_CROSS_SLOPE_BACK (15)
#define EIGHTN_CROSS_SLOPE_NEAR  (5)
#define EIGHTN_CROSS_CORNER_L    (4)
#define EIGHTN_CROSS_CORNER_R    (IMG_W - 4)
#define EIGHTN_CROSS_BREAK_DROW  (15)
#define EIGHTN_CROSS_OPEN_WIDTH  (140)
#define EIGHTN_CROSS_OPEN_ROW_MAX (IMG_H - 8)

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

/* 曾有 ROWS_CAP 行数限速表与 valid_rows 字段,已删:速度只由 Duty;
 * 丢线保护改看前瞻 look_rows==0,线回来后允许继续跑(不锁存)。 */
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

/* 低速档:实测验证可行组 Kp=2.29 Kd=1.49 Duty=2100(原含 WRef,已删除) */
#define PRESET_LOW_KP           (2.29f)
#define PRESET_LOW_KD         (1.49f)
#define PRESET_LOW_D_ALPHA    (0.40f)
#define PRESET_LOW_THRESHOLD  (0)
#define PRESET_LOW_CROSS_FILL (1)
#define PRESET_LOW_DUTY       (2100)
#define PRESET_LOW_STOP_TIME  (105)

/* 中速档:实测可用组 Kp=1.20 Kd=4.73 Duty=3200(原含 WRef,已删除)。
 * 速度全程只由菜单 Duty 决定 */
#define PRESET_MID_KP           (KP)
#define PRESET_MID_KD         (4.73f)
#define PRESET_MID_D_ALPHA    (0.40f)
#define PRESET_MID_THRESHOLD  (0)
#define PRESET_MID_CROSS_FILL (1)
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
#define PRESET_HIGH_DUTY       (STRAIGHT_DUTY)
#define PRESET_HIGH_STOP_TIME  (DRIVE_ARMED_TIMEOUT_S)

#endif /* CONFIG_H */
