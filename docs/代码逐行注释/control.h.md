# control.h 逐行注释

> 行号与源文件一致。

控制模块对外接口：PD 转向计算、占空比斜率限制、以及与菜单共享的可调增益与发车状态变量。

---

```c
// L1: 文件标识注释。
// L2: 头文件守卫起始 `#ifndef CONTROL_H`。
// L3: 定义 CONTROL_H 宏。
// L4: （空行。）
// L5: 包含标准整数类型 `<stdint.h>`。
// L6: 包含全局配置 `config.h`（舵机限位、Kp/Kd 默认值、占空比上限等）。
// L7: 包含 `image.h`，因 `control_update` 入参为 `track_info_t`。
// L8: （空行。）
// L9-L14: 定义 `control_out_t` 结构体，承载一帧控制输出：
//   L11: `servo_pwm` — 限幅后的舵机 PWM 计数值；
//   L12: `duty` — 经斜率限制后的电机占空比；
//   L13: `error_used` — 本帧实际用于 PD 的横向误差（像素，相对 IMG_CENTER）。
// L15: （空行。）
// L16: 声明全局可调 Kp，`volatile` 供菜单在另一上下文改写且主循环立即可见。
// L17: 声明全局可调 Kd。
// L18: 声明微分低通系数 `steer_d_filt_alpha`。
// L19: `drive_armed` — 菜单/按键「发车」标志，1 才允许 `cpu0_main` 驱动电机。
// L20: `drive_timed_out` — 行驶超时标志，置 1 后保持停车直至重新 arm。
// L21: `drive_stop_time_s` — 可配置的最大行驶秒数（不含发车延迟）。
// L22: `drive_duty_base` — 目标占空比基准，由菜单速度档设定。
// L23: （空行。）
// L24: `control_init` — 清零误差历史、微分滤波状态；失控恢复时也会调用。
// L25: `control_duty_reset` — 仅将内部 `g_duty_now` 置 0，用于停车/延迟而不清 PD 历史。
// L26: `control_update` — 读 `track_info_t`，写 `control_out_t`（核心 PD + duty slew）。
// L27: `control_servo_clamp` — 将舵机 PWM 钳到 [SERVO_MIN, SERVO_MAX]。
// L28: （空行。）
// L29: 结束头文件守卫 `#endif /* CONTROL_H */`。
```
