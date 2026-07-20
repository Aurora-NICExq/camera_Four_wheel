# 第 0 步：工程总览——先建立全局地图

> **目标**：知道每个文件干什么、数据怎么流、哪些宏一上车就要会改。
> 本步不读算法细节，只建立 mental model。

---

## 1. 这辆车怎么跑起来

```text
上电
  ├─ motor_hw_init：舵机中位、电机 0、按键、STM 定时器
  ├─ display_init：IPS200 竖屏 init（set_dir 在 init 之前）
  ├─ menu_init：屏幕显示调参菜单，可选从 Flash 载入参数
  ├─ startup_beep：2s 蜂鸣等待
  ├─ mt9v03x_init：相机
  └─ while(每帧)
        等 mt9v03x_finish_flag
        image_process → control_update → motor_apply
        menu_task / display_telemetry
        清 finish_flag
```

**帧是唯一时钟**：控制、失控计数、菜单以外的逻辑都用「帧数」，不用毫秒（上电等待和相机看门狗除外）。

---

## 2. 坐标约定（全工程统一）

| 概念 | 约定 |
|------|------|
| **图像行 row** | `0` = 最底行（离车最近），向上增大 |
| **相机数组** | `img[0]` = 顶行；内部用 `RAW_ROW()` 翻转 |
| **占空比 duty** | `0~10000`，对应逐飞 `PWM_DUTY_MAX` |
| **转向误差 error** | 像素：中线相对 `IMG_CENTER(94)` 的加权平均，正=偏右 |

---

## 3. 分层与依赖

```text
Layer 5  cpu0_main.c     调度 + 门闸
Layer 4  menu / display  人机（不占控制闭环）
Layer 3  control.c        转向 PD + 定速 duty
Layer 2  image.c          阈值 + 跟踪 + 误差
Layer 1  motor.c / isr.c  硬件
Layer 0  config.h pins.h  常量
```

**纯逻辑层**（可在 PC 用 gcc 编译）：`image.c`、`control.c`。  
它们只 include `<stdint.h>`、`config.h`、彼此头文件。

---

## 4. 核心数据结构

### `track_info_t`（`image.h`）

每帧图像模块的输出，约 750 字节，静态分配在 `cpu0_main.c` 的 `g_track`。

- 逐行：`left[]` / `right[]` / `mid[]` / `width[]` / `*_lost[]`
- 帧级：`valid_rows`、`error`、`both_lost_rows`、`threshold`

### `control_out_t`（`control.h`）

- `servo_pwm`：本帧舵机 duty
- `duty`：本帧电机占空比（未 armed 时为 0）
- `error_used`：本帧用于 PD 的误差（遥测用）

---

## 5. 上电必知的三道门闸

| 门闸 | 位置 | 作用 |
|------|------|------|
| `armed` | `cpu0_main.c` | 0 = 电机永远 0；菜单 Arm/Disarm 切换 |
| `DEBUG_NO_DRIVE` | `config.h` | 1 = 强制 duty=0，首次上车必须为 1 |
| 失控保护 | `cpu0_main.c` | 有效行过少 / 全白 → 断油 + 长鸣 |

---

## 6. 目录

根目录即全部业务源码（`.c/.h`）与 `docs/` 文档，拷到 ADS 工程的 `user/` 即可。

---

## 7. 自查

- [ ] 能画出：摄像头 → image → control → motor 的数据流
- [ ] 知道 `row=0` 是图像最底行
- [ ] 知道谁可以写 PWM（只有 `motor.c`）
- [ ] 知道上电默认 `armed=0` 且 `DEBUG_NO_DRIVE=1`
- [ ] 知道本工程**没有** FSM / 编码器 / 速度 PID

下一步：[配置与引脚](./step1_配置与引脚.md)
