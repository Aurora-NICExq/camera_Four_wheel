# cpu0_main.c 逐行注释

> 行号与源文件一致。

CPU0 主循环：菜单任务 → 等待摄像头帧 → 图像巡线 → 失控保护 → PD 控制 → 按模式/发车状态驱动电机 → 可选调试叠显。双核中所有实时控制链均在此核执行。

---

## 头文件与内存段

```c
// L1: 文件标识注释。
// L2: 全局配置常量。
// L3: 控制模块（PD、`drive_armed`、超时）。
// L4: 图像处理（`image_process`、`track_info_t`）。
// L5: 菜单逻辑与参数调节。
// L6: 电机/舵机硬件抽象。
// L7: 菜单与 IPS 屏、按键等板级移植层。
// L8: 逐飞公共头（摄像头 `mt9v03x_*`、IPS200 等）。
// L9: （空行。）
// L10: `#pragma section all "cpu0_dsram"` — 将后续静态/全局变量链接到 CPU0 专用 DSRAM 段（双核内存隔离）。
// L11: （空行。）
// L12: 静态全局 `g_track`，保存每帧巡线结果（误差、前瞻行数、阈值等）。
```

---

## core0_main 初始化

```c
// L14: CPU0 入口函数，由启动代码调用。
// L15: `clock_init()` — 系统时钟配置。
// L16: `debug_init()` — 调试串口/日志（若启用）。
// L17: （空行。）
// L18: `motor_hw_init()` — PWM、GPIO、默认停车。
// L19: `menu_port_init()` — 按键、屏幕等菜单硬件。
// L20: `control_init()` — PD 状态与占空比斜坡清零。
// L21: `menu_init()` — 菜单状态机与默认参数。
// L22: （空行。）
// L23: `mt9v03x_init()` — MT9V03X 摄像头初始化。
// L24: `cpu_wait_event_ready()` — 等待双核同步事件（与 CPU1 同时就绪后再进主循环）。
```

---

## 主循环局部变量

```c
// L26: `fail_cnt` — 失控保护计数器，连续「前瞻无行」帧数。
// L27: `armed_elapsed_us` — 自首次 armed 以来累计行驶时间（微秒），含延迟与行驶段。
// L28: `armed_last_us` — 上一帧采样 `hal_time_us()` 的时间戳，用于算 dt。
// L29: `armed_t0_set` — 是否已建立 armed 计时起点（首帧只置位不计 dt）。
// L30: `drive_en` — 本帧是否允许驱动电机；失控保护可拉低，有效巡线可恢复。
// L31: `out` — `control_out_t` 控制输出，初值全 0。
// L32: （空行。）
// L33: `while (TRUE)` — 永不退出的主循环。
```

---

## 菜单与帧同步

```c
// L34: 每圈先跑菜单（按键扫描、参数修改、模式切换）。
// L35: （空行。）
// L36-L38: 若摄像头 DMA 未完成（`mt9v03x_finish_flag==0`），空转等待下一帧，不做图像处理。
// L39: （空行。）
```

---

## 图像处理

```c
// L40: 注释：图像处理段。
// L41: （空行。）
// L42: 将摄像头缓冲区转为 `uint8_t[IMG_H][IMG_W]` 指针，调用 `image_process` 填充 `g_track`（八邻域、前瞻误差、十字等）。
// L43: （空行。）
```

---

## 失控保护（failsafe）

前瞻滑窗有效行数 `look_rows==0` 表示当前帧完全没有可用边线信息。连续达到 `FAILSAFE_FRAMES`（2）帧则禁止驱动；一旦恢复有效行则立即清零计数并重新允许驱动。

```c
// L44: 开独立代码块（限制 fail_cnt 等作用域）。
// L45: （空行。）
// L46: 注释：失控保护。
// L47: （空行。）
// L48: 若本帧前瞻参与行数为 0（完全丢线/无有效瞄准）。
// L49-L51: 失败计数递增，但不超过 FAILSAFE_FRAMES，避免无符号溢出。
// L52-L55: 否则（有有效前瞻行）：清零 fail_cnt，并置 `drive_en=1` 允许驱动。
// L56-L58: 若连续失败已达 2 帧，强制 `drive_en=0`，后续走停车分支。
// L59: 结束失控保护代码块。
// L60: （空行。）
```

---

## PD 控制

```c
// L61: 无论是否允许驱动，每帧都更新 PD（保持舵机演算连续；实际 PWM 由下游 motor 分支决定是否输出）。
```

---

## 电机输出分支

优先级：电机测试 > 左轮测试 > 舵机对中测试 > 正常发车 > 默认停车。

```c
// L63: 注释：电机处理。
// L64: （空行。）
// L65: 若菜单 Motor Test：舵机中位 + 固定 MOTOR_TEST_DUTY（2000），忽略巡线与 armed。
// L66: `motor_apply(SERVO_CENTER, MOTOR_TEST_DUTY)`。
// L67: 否则若 Left Test：仅左轮 2000，舵机回中。
// L68: `motor_apply_left_only(MOTOR_TEST_DUTY)`。
// L69: 否则若 Align Test：停车后只跟 `out.servo_pwm`，用于看转向响应。
// L70: `motor_apply_servo_only(out.servo_pwm)`。
// L71: 否则若 `drive_en && drive_armed` — 正常比赛发车路径。
```

### 正常发车：延迟、超时、行驶

```c
// L72: 取当前微秒时间戳。
// L73-L76: 首帧 armed：只设 `armed_t0_set=1`，`armed_elapsed_us` 保持 0（延迟从第二帧开始累加）。
// L77-L81: 后续帧：dt = now - last；若 dt > 200ms（掉帧），钳为 20ms 名义间隔；累加到 `armed_elapsed_us`。
// L82: 更新 `armed_last_us` 供下帧使用。
// L83: （空行。）
// L84-L89: 超时判定：`drive_timed_out` 已置位，或累计时间 ≥ 发车延迟(2s) + drive_stop_time_s；
//          则置 `drive_timed_out=1`，`motor_reset()` 停车回中，`control_duty_reset()` 占空比斜坡清零。
// L90-L93: 否则若仍在发车延迟 2 秒内：只停车不回清 PD（`control_duty_reset` 清斜坡，延迟结束后再爬升 duty）。
// L94-L95: 否则（延迟已过且未超时）：正常 `motor_apply(out.servo_pwm, out.duty)`。
// L96-L103: 未满足发车条件（未 armed、失控、等）：`motor_reset()` 停车；
//            若因失控 `!drive_en` 则 `control_init()` 全清 PD 状态；
//            若仅未 armed 则只 `control_duty_reset()` 保留 PD 历史。
```

### armed 状态复位

```c
// L105-L108: 当菜单取消 armed（`!drive_armed`）时，清除计时起点与超时标志，下次发车重新计时。
// L109: （空行。）
```

---

## Camera 调试叠显

仅在菜单 Camera View 开启时，在 IPS200 上于图像下方显示遥测字段。

```c
// L110: 若菜单处于摄像头调试视图。
// L111: `image_debug_show(&g_track)` — 在屏上绘制边线/前瞻等叠加。
// L112: 第 1 行标签 "ERR"。
// L113: 显示 `out.error_used` 横向误差，4 位宽，列 32。
// L114: 标签 "SRV"。
// L115: 显示舵机 PWM，列 136。
// L116: 第 2 行标签 "LOOK"。
// L117: 显示前瞻有效行数 `g_track.look_rows`。
// L118: 标签 "DTY"。
// L119: 显示当前占空比 `out.duty`。
// L120: 第 3 行标签 "LST"。
// L121: 显示双边丢线行数 `both_lost_rows`。
// L122: 标签 "TH"。
// L123: 显示本帧二值化阈值。
// L124: 标签 "CRS"。
// L125: 显示十字补线是否有效 `cross_valid`（0/1）。
// L126: 第 4 行标签 "HLD"。
// L127: 显示误差保持计数 `err_hold`。
// L128: 标签 "FAR"。
// L129: 显示菜单设定的 `steer_look_far` 前瞻顶点行。
// L130: 结束 Camera View 分支。
// L131: 清除帧完成标志，释放缓冲区给 DMA 采集下一帧。
// L132: 主循环结束 `}`。
// L133: `core0_main` 结束 `}`（实际不可达）。
```

---

## 内存段恢复

```c
// L135: `#pragma section all restore` — 恢复默认链接段，避免影响后续编译单元。
```

---

## 控制链时序图

```
帧到达 → image_process → failsafe(drive_en)
                      → control_update(PD + duty slew)
                      → motor_apply / reset（按模式与 armed 状态）
```

| 条件 | 电机行为 |
|------|----------|
| look_rows==0 连续 2 帧 | drive_en=0，停车，control_init |
| armed 且前 2s | motor_reset，duty 斜坡清零 |
| armed 且超时 | drive_timed_out，永久停车直至重新 arm |
| armed 且正常 | motor_apply(servo, duty) |
