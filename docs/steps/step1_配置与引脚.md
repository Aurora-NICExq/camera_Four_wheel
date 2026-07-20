# 第 1 步：配置与引脚——所有参数的单一来源

> **目标**：读懂 `config.h` 和 `pins.h`，知道改哪几个宏就能影响行为。
> 对应文件：`config.h`、`pins.h`

---

## 1. 为什么拆成两个文件

| 文件 | 内容 | 谁能 include |
|------|------|--------------|
| `config.h` | 算法参数、安全阈值、显示分频 | 纯逻辑层 + 全工程 |
| `pins.h` | MCU 引脚、IPS200 接口类型 | 仅硬件层（`motor.c`、`display.c` 等） |

`config.h` **不得** include 逐飞头文件——这样 `image.c` 才能在 PC 上离线编译。

---

## 2. 图像与帧率（`config.h` §一）

```c
#define IMG_W           (188)
#define IMG_H           (120)
#define IMG_CENTER      (94)      /* 转向误差零点 */
#define FRAMES_PER_SECOND (50)    /* 与 MT9V03X_FPS_DEF 一致 */
```

`motor.c` 内有编译检查：`IMG_W/H` 必须等于 `MT9V03X_W/H`。

**运行时阈值**：不在 `config.h` 写死，而在 `image.c` 的 `volatile image_threshold`：
- `0` → 自动大津法（默认）
- `>0` → 手动固定阈值（菜单 `Threshold` 项）

---

## 3. 边线与 18th 跟踪（§二）

关键宏：

| 宏 | 典型值 | 含义 |
|----|--------|------|
| `WIDTH_MIN_PX` / `WIDTH_MAX_PX` | 20 / 186 | 合法赛道宽度范围 |
| `TH18_COL_MARGIN` | 20 | 最长白列搜索左右留白 |
| `TH18_CROSS_BOTH_LOST_MIN` | 10 | 十字补线触发：双边丢线行数下限 |
| `STEER_W_BANDS` | 8 | 误差加权行带数 |

两套转向权重表 `STEER_WEIGHTS_LOWSPEED` / `STEER_WEIGHTS_HIGHSPEED`：占空比越高，权重越向**远端行**移动（提前看弯）。

---

## 4. 转向与舵机（§三）

```c
#define SERVO_CENTER    (770)     /* 直行中位，实车标定 */
#define SERVO_MIN       (685)     /* 机械左极限 */
#define SERVO_MAX       (850)     /* 机械右极限 */
#define SERVO_DIR       (+1)      /* 装反了改 -1 */
#define SERVO_SLEW_LIMIT (45)     /* 每帧舵机最大变化 */
```

PD 默认：`KP_MIN/MAX`、`KD`、`D_FILT_ALPHA` 等。菜单可运行时改 `control.c` 里的 `volatile` 副本。

---

## 5. 速度策略（§四，定速开环）

```c
#define STRAIGHT_DUTY   (4500)    /* 直道/弯道统一基准 */
#define DUTY_HARD_CAP   (6000)    /* 全局硬上限 60% */
#define DUTY_SLEW_UP    (120)     /* 升斜坡 → 兼作软启动 */
#define DUTY_SLEW_DOWN  (10000)   /* 降斜坡，默认不限 */
```

无编码器 → **不能**做闭环速度；减速靠滑行 + 占空比斜坡。

---

## 6. 安全（§五）

| 宏 | 作用 |
|----|------|
| `FAILSAFE_MIN_ROWS` | 有效行 < 8 → 图像失效 |
| `FAILSAFE_FRAMES` | 连续 10 帧失效 → 断油 |
| `FAILSAFE_SEVERE_FRAMES` | 严重全白 2 帧 → 快速断油 |
| `CAMERA_WATCHDOG_US` | 武装后 100ms 无帧 → 断油 |
| `DEBUG_NO_DRIVE` | **首次上车必须为 1** |

---

## 7. 显示（§七）与竖屏约束

```c
#define DISPLAY_IMG_W   (94)      /* 若将来画图像预览，不得超过 240 */
#define DISPLAY_IMG_H   (60)
#if (DISPLAY_IMG_W > 240) || (DISPLAY_IMG_H > 320)
#error "IPS200 竖屏 240x320：DISPLAY_IMG_W/H 不得超过 240/320"
#endif
```

菜单几何在 `menu_port.h`：`MENU_COLS=30`、`MENU_ROWS=20`（240/8 × 320/16）。

---

## 8. `pins.h` 引脚

```c
#define PIN_SERVO_PWM       (ATOM1_CH1_P33_9)   /* 50Hz，ATOM1 */
#define PIN_MOTOR1_PWM      (ATOM0_CH2_P21_4)   /* ~17kHz，ATOM0 */
#define PIN_MOTOR1_DIR      (P21_5)
#define PIN_MOTOR2_PWM      (ATOM0_CH4_P02_4)
#define PIN_MOTOR2_DIR      (P02_5)
#define PIN_BUZZER          (P33_10)
#define IPS200_CONNECT_TYPE (IPS200_TYPE_SPI)
```

约束：**舵机与电机 PWM 必须在不同 ATOM 模块**（频率差两个数量级）。

---

## 9. 上车前建议改的宏

1. `DEBUG_NO_DRIVE` → 先 `1`，验收后再 `0`
2. `SERVO_DIR` → 装车后直道微调
3. `STRAIGHT_DUTY` / `DUTY_HARD_CAP` → 按电池与赛道摩擦标定
4. `SERVO_MIN/MAX/CENTER` → 按实际舵机机械行程

---

## 10. 自查

- [ ] 能说出 `config.h` 与 `pins.h` 的分工
- [ ] 知道 `image_threshold=0` 与手动阈值的区别
- [ ] 知道 `DEBUG_NO_DRIVE` 和 `armed` 是两道独立的断油门闸
- [ ] 知道竖屏菜单是 30 列而不是 40 列

下一步：[图像流水线](./step2_图像流水线.md)
