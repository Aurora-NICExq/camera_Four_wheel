# 第 4 步：电机、显示与菜单——硬件 HAL 与竖屏调参

> **目标**：理解唯一写 PWM 的模块、IPS200 竖屏初始化顺序、菜单如何改参数。
> 对应文件：`motor.c/h`、`display.c/h`、`menu*.c/h`

---

## 1. motor.c — 硬件执行层

### 初始化（`motor_hw_init`）

```text
system_start()           → hal_time_us() 时基
pwm_init 舵机 50Hz 中位
pwm_init 两路电机 17kHz 占空比 0
gpio_init 两路 DIR 为前进电平
gpio_init 蜂鸣器
key_init(1000/FRAMES_PER_SECOND)
```

上电瞬间：**舵机中位、电机 0**，防止乱转。

### 每帧输出

```c
motor_apply(servo_pwm, duty);   /* 正常 */
motor_stop();                    /* 断油 */
```

`motor_apply` 内舵机再过 `control_servo_clamp`；两路电机 PWM **始终相同**（无电子差速）。

### HAL 薄封装

| 函数 | 用途 |
|------|------|
| `hal_time_us()` | 相机看门狗 |
| `hal_key_scan()` | 由 `menu_task` 间接调用 |
| `hal_buzzer_on/off()` | 提示音 |

---

## 2. display.c — 屏幕 init + 遥测 + 蜂鸣

**屏幕绘制归属菜单**；`display.c` 不做菜单 UI。

### 竖屏初始化（关键顺序）

```c
ips200_set_dir(IPS200_PORTAIT);   /* 必须在 init 之前！ */
ips200_init(IPS200_CONNECT_TYPE);
ips200_clear();
wireless_uart_init();
```

SeekFree 库：`ips200_set_dir` 在 `ips200_init` **之后**调用无效。竖屏 `width_max=240`，`height_max=320`。

### 每帧

- `display_update()`：蜂鸣器帧计数
- `display_telemetry()`：无线串口一行（`TELEMETRY_DIV` 限流）

格式：`F<帧> E<误差> R<有效行> D<占空比>`

---

## 3. 菜单系统架构

```text
menu_config.c   定义 menu_items[]（绑 volatile 全局）
       ↓
menu.c          引擎：光标、编辑、Flash 校验和
       ↓
menu_port.c     唯一 include 逐飞头文件：ips200 绘制、按键、Flash 页
```

### 竖屏几何（`menu_port.h`）

```c
#define MENU_COLS   (30)    /* 240 / 8 */
#define MENU_ROWS   (20)    /* 320 / 16 */
```

`menu.c` 中 `VALUE_COL=18`，左侧标签、右侧数值，避免 x≥240 触发 `zf_assert`。

### 按键映射（`menu_port.c`）

| 键 | 功能 |
|----|------|
| KEY_1 | UP |
| KEY_2 | DOWN |
| KEY_3 | ENTER |
| KEY_4 | BACK |

长按 UP/DOWN 自动重复（10× 步长）。

### 菜单项一览（`menu_config.c`）

- **SteerPID**：Kp/Kd 相关 7 项
- **Threshold**：二值化（0=大津）
- **Arm/Disarm**：切换 `armed`（不写 Flash）
- **Save / Load / Restore Def**：Flash 持久化

### Flash 格式

`magic + version + count + params + checksum`，sector 0 page 8（与逐飞 demo 一致）。改菜单项后须 bump `MENU_FLASH_VERSION`（`menu.h`）。

---

## 4. 模块边界

| 模块 | 可以做 | 不可以做 |
|------|--------|----------|
| motor.c | 写 PWM/GPIO | 算 PD、判图像 |
| display.c | init 屏幕、遥测、蜂鸣 | 画菜单 UI |
| menu.c | 改 volatile 全局 | 直接 `motor_apply` |
| menu_port.c | ips200/Flash/按键 | 业务算法 |

---

## 5. 自查

- [ ] 能说出 `ips200_set_dir` 必须在 `init` 前的理由
- [ ] 知道菜单 30 列与旧教程 40 列的区别
- [ ] 知道 Arm/Disarm 在 `cpu0_main` 里消费 `g_arm_toggle_req`
- [ ] 知道 Save 存的是哪些变量（不含 Arm 状态）

下一步：[主循环与安全门闸](./step5_主循环与安全门闸.md)
