# 第 3 步：控制律——转向 PD 与定速开环

> **目标**：理解 `control_update()` 如何从 `track_info.error` 得到舵机和电机输出。
> 对应文件：`control.c`、`control.h`

---

## 1. 接口

```c
typedef struct {
    uint16_t servo_pwm;
    uint16_t duty;
    int16_t  error_used;
    uint16_t duty_target;
} control_out_t;

void control_update(const track_info_t *ti, uint8_t armed, control_out_t *out);
```

**纯逻辑**：无 `#include "zf_common_headfile.h"`。

---

## 2. 菜单可调参数（`volatile` 全局）

`menu_config.c` 里的项直接绑定这些变量，改完下一帧生效：

| 变量 | 菜单名 | 作用 |
|------|--------|------|
| `steer_kp_min/max/e_sat` | Kp Min/Max/E Sat | 误差大时 Kp 二次插值 |
| `steer_use_const_kp` | Use Const Kp | 1=恒定 Kp |
| `steer_kp_const` | Kp Const | 恒定 Kp 值 |
| `steer_kd` | Kd | 微分 |
| `steer_d_filt_alpha` | D Filt Alpha | D 项 EMA |

Flash Save/Load 持久化的是这些值（不含 Arm/Disarm）。

---

## 3. 转向 PD

```text
error = ti->error
d_raw = error - g_prev_error
g_d_filt += alpha * (d_raw - g_d_filt)    /* EMA 滤波 */

kp = f(|error|)   /* 调度或恒定 */

servo_raw = SERVO_CENTER + SERVO_DIR * (kp*error + kd*g_d_filt)
```

### Kp 调度

```text
ratio = min(|error| / KP_E_SAT, 1)
kp = KP_MIN + (KP_MAX - KP_MIN) * ratio²
```

零误差附近 Kp 小（直道不画龙），大误差 Kp 大（弯道跟线）。

### 舵机限速

目标 `servo_target` 与上一帧 `g_servo_now` 差值限制在 `SERVO_SLEW_LIMIT`，防一帧打满。

### 双道钳制

1. `control.c` 内 `control_servo_clamp`
2. `motor_apply()` 写入前再钳一次

---

## 4. 速度（定速开环）

```text
target = min(STRAIGHT_DUTY, DUTY_HARD_CAP)
g_duty_now 向 target 沿 DUTY_SLEW_UP/DOWN 爬升
若 !armed → g_duty_now = 0
out->duty = g_duty_now
```

**没有**行数减速、曲率减速、boost——精简版直道+弯道统一基准速度。

`control_init()` 在解锁时由 `cpu0_main` 调用，清零 PD 记忆与斜坡状态。

---

## 5. `armed` 与控制的关系

| `armed` | 舵机 | 电机 duty |
|---------|------|-----------|
| 0 | 仍跟踪（可观察转向） | 强制 0 |
| 1 | 跟踪 | 沿斜坡爬升到 target |

注意：`DEBUG_NO_DRIVE=1` 时 main 在 `motor_apply` 前把 duty 置 0，与 `armed` 独立。

---

## 6. 内部状态（跨帧）

| 变量 | 重置时机 | 作用 |
|------|----------|------|
| `g_prev_error` | `control_init` | D 项 |
| `g_d_filt` | `control_init` | D 项滤波 |
| `g_duty_now` | `control_init` / 每帧斜坡 | 实际占空比 |
| `g_servo_now` | 每帧 slew | 实际舵机 |

---

## 7. 调参顺序建议

1. `Use Const Kp=1`，固定 Kp，先调 `Kd` 和 `D Filt Alpha` 压画龙
2. 关闭恒定 Kp，调 `Kp Min/Max/E Sat` 兼顾直道与弯道
3. `STRAIGHT_DUTY` 在 `config.h`，菜单不暴露——改宏重新编译
4. 舵机方向反了 → 改 `SERVO_DIR` 或机械安装

---

## 8. 自查

- [ ] 能写出 PD 公式和 Kp 调度形状（二次）
- [ ] 知道 `g_duty_prev` 在 main 里来自门闸后的 duty，反馈给 image 权重
- [ ] 知道 `control_init` 何时被调用（解锁时）
- [ ] 知道本模块不写 PWM

下一步：[电机、显示与菜单](./step4_电机显示与菜单.md)
