/*********************************************************************************************************************
 * 文件：config.h — 全部可调参数（唯一存放处，.c 文件中禁止出现魔法数字）
 *
 * 本文件只含宏定义，不 include 任何 MCU/逐飞头文件 —— image.c/control.c 依赖它，
 * 必须保证在 PC (gcc) 上可直接编译（test/replay.c 离线回放）。
 *
 * 坐标/单位约定（全工程统一）：
 *   - 行号 row：0 = 图像最底行（离车最近），向上递增；
 *   - 占空比 duty：uint16_t，0~10000（对应逐飞 PWM_DUTY_MAX）；
 *   - 所有时间一律用"帧"为单位（FRAMES_PER_SECOND 换算），绝不用毫秒 ——
 *     主循环严格帧同步，帧就是本系统唯一可靠的时钟。
 ********************************************************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

/*========================================== 一、图像 / 帧率 ========================================================*/

#define IMG_W                   (188)   /* 图像宽（列数）。必须与 MT9V03X_W 一致（motor.c 内有编译期检查）      */
#define IMG_H                   (120)   /* 图像高（行数）。必须与 MT9V03X_H 一致                                */
#define IMG_CENTER              (94)    /* 图像中心列 = IMG_W/2，转向误差的零点                                 */

#define FRAMES_PER_SECOND       (50)    /* 帧率。与 zf_device_mt9v03x.h 的 MT9V03X_FPS_DEF(50) 一致。
                                           所有 *_FRAMES 宏都以此为时基；若改帧率，所有帧数宏都要重新标定！    */

#define COL_STEP                (1)     /* 列抽样步长。流水线耗时超预算时先把它提到 2（精度减半、耗时减半），
                                           再考虑降帧率 —— 降帧率会拉长控制延迟，代价更大                     */

/* 二值化阈值改由菜单运行时可调（image.c 的 volatile image_threshold）：
 *   image_threshold == 0 → 自动大津法（默认，行为与精简前一致）；
 *   image_threshold >  0 → 手动固定阈值（强阴影/灯光频闪时的保底方案，菜单里现调现看）。         */
#define FIXED_THRESHOLD         (128)   /* 大津法内部下限兜底 & 手动模式的典型参考值                            */
#define OTSU_ROW_STEP           (2)     /* 大津法直方图统计的行抽样步长（阈值不需要全像素统计，省一半时间）     */
#define OTSU_COL_STEP           (2)     /* 大津法直方图统计的列抽样步长                                         */
#define OTSU_THRESHOLD_MIN      (40)    /* 大津结果下限保护：全黑/全白画面时大津会漂到荒谬值                    */
#define OTSU_THRESHOLD_MAX      (200)   /* 大津结果上限保护                                                     */

/*========================================== 二、边线搜索 / 中线 ====================================================*/

#define WIDTH_MIN_PX            (20)    /* 合法赛道宽度下限（像素）：小于此认为该行不可信                       */
#define WIDTH_MAX_PX            (186)   /* 合法赛道宽度上限                                                     */
#define WIDTH_TABLE_DEFAULT     (110)   /* 某行完全没有学习来源时的兜底半径基数（近处赛道典型宽度，需实测校准） */

/* 最长白列起种 + 八邻域双边融合跟踪（hybrid_track.c）。 */
#define HYBRID_EDGE_REACQUIRE_RADIUS    (8)  /* 边点严格八邻域断开后的预测搜索半径（像素）                       */
#define HYBRID_SEED_REACQUIRE_RADIUS    (6)  /* 中线白种子断开后的预测搜索半径（像素）                           */
#define HYBRID_MAX_GAP_ROWS             (4)  /* 最多跨越的连续全黑/无边行；再长就停止，避免无依据补线             */
#define HYBRID_MAX_ROW_SHIFT            (6)  /* 单行斜率预测限幅；只限制预测窗口，不钳制真实测得的中线             */

/* 行置信分级（白种子行 ≠ 边线实测行）：
 *   双边实测 > 单边重建 > 双边丢失预测。双边丢失但种子仍在白区内的行只证明"前方可行驶"，
 *   不证明"边线在哪"——这类行的中线跟随种子（逐行实测的白像素）而不是沿旧斜率外推，
 *   彻底避免大片白区里凭两行斜率画出几十行的假想对角线。                                             */
#define HYBRID_MAX_PREDICT_ROWS         (8)  /* 锚定白列长度之外，连续双边丢失行的最大跟踪预算；超出即停止         */
#define HYBRID_PREDICT_MAX_STEP_PX      (2)  /* 重捕获被拒后，走廊中线向候选边每行允许收敛的最大像素数             */
/* 重捕获连续性闸门：断线后（或单边行）新边给出的中线与走廊中线偏差超过此值 → 本行拒收，
 * 保持预测行身份。默认取"单行最大预测横移 + 重捕获窗口半径"——正常急弯的合法跳变上限。      */
#ifndef HYBRID_REACQUIRE_MAX_JUMP_PX
#define HYBRID_REACQUIRE_MAX_JUMP_PX    (HYBRID_MAX_ROW_SHIFT + HYBRID_EDGE_REACQUIRE_RADIUS)
#endif

/* 转向误差 = 各行 (mid - IMG_CENTER) 的加权平均。8 个行带（每带 IMG_H/8=15 行，带 0 最近）。
 * 为什么要两张表并按占空比混合：无编码器 + 只能滑行 → 速度越高必须看得越远越早动作，
 * 低速表近重（稳定不画龙），高速表远移（提前入弯）。混合系数 = duty / DUTY_HARD_CAP ∈ [0,1]。      */
#define STEER_W_BANDS           (8)                                 /* 行带数量                                */
#define STEER_W_BAND_ROWS       (15)                                /* 每带行数 = IMG_H / STEER_W_BANDS        */
#define STEER_WEIGHTS_LOWSPEED  { 8, 10, 9, 6, 4, 2, 1, 0 }         /* 低速：近端最重，最远带不参与            */
#define STEER_WEIGHTS_HIGHSPEED { 2,  4, 6, 9, 10, 8, 5, 2 }        /* 高速：权重整体远移，远端也有话语权      */

/* 行置信权重缩放（百分比）：误差只应由真实测量主导。
 * 双边实测行全权重；单边重建行半权重（另一边靠宽度表推算）；
 * 双边丢失预测行零权重——它们本来就是由下方实测行推出来的，重复计入只会放大伪影。          */
#define STEER_W_SINGLE_EDGE_PCT (50)    /* 单边重建行权重百分比                                                 */
#define STEER_W_BOTH_LOST_PCT   (0)     /* 双边丢失预测行权重百分比                                             */

/* 精简版（直道+转弯）：无元素检测器、无状态机、无曲率估计 —— image.c 只算加权中线误差，
 * control.c 直接对其做 PD。原 §三 元素检测器 / §四 状态机 / 曲率相关宏已随 fsm.c 一并删除。   */

/*========================================== 三、转向（control.c） ==================================================*/

/* 舵机 PWM 换算：50 Hz → 周期 20000 µs；duty 满量程 10000 → 1 duty = 2 µs。
 * ！！SERVO_MIN/SERVO_MAX 是机械极限（打死前的最大偏移），已按本车实测标定 —— 超程会打坏舵机齿轮！！
 * 中位/极限已固定为常量（不再进菜单调，Servo 页已删除）。                                          */
#define SERVO_PWM_HZ            (50)    /* 舵机 PWM 频率（标准 RC 舵机）                                        */
#define SERVO_CENTER            (770)   /* 直行中位 duty（实车标定）                                            */
#define SERVO_MIN               (685)   /* 左极限 duty（机械打死保护下限，实车标定）                            */
#define SERVO_MAX               (850)   /* 右极限 duty（机械打死保护上限，实车标定）                            */
#define SERVO_DIR               (+1)    /* 极性：+1 = 正误差（中线偏右）打右舵；装车实测方向不对就改 -1         */
#define SERVO_SLEW_LIMIT        (45)    /* 舵机 PWM 每帧最大变化，防止分叉误判时一帧横跨全部机械行程             */

/* PD 参数。Kp 随 |误差| 调度（二次插值）：零点附近小（高速直道不画龙），大误差时大（弯道打舵狠）。 */
#define USE_CONST_KP            (0)     /* 1 = 使用恒定 Kp（调试基线），0 = 使用调度 Kp                         */
#define KP_CONST                (1.8f)  /* 恒定 Kp 模式的值                                                     */
#define KP_MIN                  (1.0f)  /* 调度 Kp：|e|=0 处                                                    */
#define KP_MAX                  (3.2f)  /* 调度 Kp：|e|≥KP_E_SAT 处                                             */
#define KP_E_SAT                (40.0f) /* Kp 达到最大值的误差幅度（像素）                                      */
#define KD                      (6.0f)  /* 微分系数（对滤波后的 d_filt）                                        */
#define D_FILT_ALPHA            (0.4f)  /* D 项 EMA 系数：d_filt += α(d_raw − d_filt)。像素量化误差使原始差分
                                           充满毛刺，不滤波的 D 项只会放大噪声                                  */

/*========================================== 四、速度策略（control.c，定速开环） ====================================*/

/* 精简版：去掉行数/曲率/转向减速表、boost、出弯再加速门闸。仅保留基准占空比 + 硬上限 + 斜坡。
 * 以下三项已固定为常量（Speed 菜单页已删除，不再运行时可调）。                                     */

#define STRAIGHT_DUTY           (4500)  /* 直道/弯道统一基准占空比（固定）                                      */
#define DUTY_HARD_CAP           (6000)  /* 全局硬上限 = 满量程 10000 的 60%（硬性封顶）                          */

/* 占空比斜坡（每帧最大变化量）：
 * 降斜坡默认 10000 = 不限幅；升斜坡小步长防打滑，兼作解锁后软启动。                              */
#define DUTY_SLEW_DOWN          (10000) /* 每帧最大下降量（默认不限）                                           */
#define DUTY_SLEW_UP            (120)   /* 每帧最大上升量（固定，50fps 下 0→4500 约 0.75 s）                    */

/* 硬件挂钩（本车没有的功能，恒 0，代码编译剔除）：                                                 */
#define ENABLE_HW_BRAKE         (0)     /* 驱动板刹车脚：未接线。任何减速逻辑不得依赖它                         */
#define ENABLE_VBAT_COMP        (0)     /* 电池电压补偿：本板无分压 ADC（已确认），无法实现。                    */

/*========================================== 五、安全 / 解锁 ========================================================*/

#define FAILSAFE_MIN_ROWS       (8)     /* 有效行数 < 此值视为"看不见赛道"（全黑/冲出）                         */
#define FAILSAFE_MAX_BOTH_LOST_PCT (70) /* 双边同时丢失比例达到此值也视为图像失效（覆盖全白/严重过曝）          */
#define FAILSAFE_SEVERE_BOTH_LOST_PCT (90) /* 严重全白阈值，使用更短确认时间                                  */
#define FAILSAFE_FRAMES         (10)    /* 一般图像失效连续帧数 → 占空比 0 并解除武装                          */
#define FAILSAFE_SEVERE_FRAMES  (2)     /* 严重全白连续帧数 → 快速断油，避免在高占空比下盲冲                    */
#define CAMERA_WATCHDOG_US      (100000u) /* 武装后 100 ms 没有完整帧 → 立即断油；时基独立于相机帧             */
#define STARTUP_DELAY_MS        (2000)  /* 上电后蜂鸣等待时间（唯一的毫秒计时：发生在相机启动前，尚无帧时基）   */
#define STARTUP_BEEP_MS         (100)   /* 上电提示音单次时长                                                   */

#define DEBUG_NO_DRIVE          (1)     /* 1 = 调试模式：完整流水线+丰富显示，占空比强制 0（首次上车必须为 1）  */

/*========================================== 六、滑行特性标定（TEST_COAST） =========================================*/

#define TEST_COAST              (0)     /* 1 = 滑行标定模式：直道恒速巡航，按键或黑标记线触发切油，
                                           每帧串口输出 (frame, valid_rows, duty)。流程见 README 第 5 步        */
#define TEST_COAST_CRUISE_DUTY  (4000)  /* 标定巡航占空比（每轮标定改这个值）                                   */
#define TEST_COAST_TARGET_DUTY  (0)     /* 触发后的目标占空比（测全滑行填 0）                                   */
#define TEST_COAST_MARKER_ROWS  (30)    /* 有效行数骤降到此值以下 = 检测到横贯赛道的黑标记条（自动触发切油）    */

/*========================================== 七、显示 / 遥测（display.c） ===========================================*/

#define MOTOR_PWM_HZ            (17000) /* 电机 PWM 频率：>16 kHz 人耳不可闻；与舵机必须在不同 ATOM 模块        */
#define MOTOR_DIR_FORWARD_LEVEL (0)     /* 电机 DIR 前进电平：0 = 低电平正转（本车实测）。两路 DIR 上电即置此值 */

#define DISPLAY_TEXT_DIV        (25)    /* 文本页刷新分频：每 25 帧一次 = 2 Hz（比赛模式的唯一屏幕开销）        */
#define DISPLAY_IMG_DIV         (10)    /* 调试模式图像页刷新分频：每 10 帧一次                                 */
#define DISPLAY_IMG_W           (94)    /* 图像页显示宽（188/2，降采样一半，减少 SPI 传输量）                   */
#define DISPLAY_IMG_H           (60)    /* 图像页显示高（120/2）                                                */
#if (DISPLAY_IMG_W > 240) || (DISPLAY_IMG_H > 320)
#error "IPS200 竖屏 240x320：DISPLAY_IMG_W/H 不得超过 240/320"
#endif
#define ENABLE_UART_TELEMETRY   (1)     /* 1 = 无线串口遥测（比赛模式下也按 TELEMETRY_DIV 限流）                */
#define TELEMETRY_DIV           (25)    /* 遥测行分频（TEST_COAST 模式下自动改为每帧）                          */
#define ENABLE_UART_IMAGE       (0)     /* 1 = 串口回传整帧图像（仅 DEBUG_NO_DRIVE=1 时编译；分块非阻塞：
                                           每帧只发半行 94 字节 ≈8 ms@115200，整帧约 4.8 s —— 静态调试用）      */

/* 蜂鸣器状态提示音时长（帧）                                                                        */
#define CHIRP_FRAMES_SHORT      (3)     /* 短鸣（保留）                                                         */
#define CHIRP_FRAMES_MID        (8)     /* 中鸣：坡道 / 恢复                                                    */
#define CHIRP_FRAMES_LONG       (20)    /* 长鸣：故障 / 失控保护                                                */

/* 按键功能分配（索引对应 zf_device_key.h 的 KEY_LIST）
 * 常规运行时 4 个按键由调试菜单独占：KEY_1..KEY_4 = UP/DOWN/ENTER/BACK（见 menu_port.c）；
 * 解锁改由菜单 System 页的 "Arm/Disarm" 动作触发（cpu0_main.c）。
 * 下面仅 TEST_COAST 标定模式（默认关闭）会自行读键，其它索引已不再使用。 */
#define KEY_IDX_COAST           (2)     /* TEST_COAST 手动触发切油（仅 TEST_COAST=1 时使用）                    */

#endif /* CONFIG_H */
