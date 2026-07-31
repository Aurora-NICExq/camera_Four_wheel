# motor.c 逐行注释

> 行号与源文件一致。

将 `control_out_t` 中的舵机 PWM 与占空比映射到 TC264 的 PWM 引脚。左右轮采用「IN1 正向 PWM、IN2 恒 0」的驱动方式；`hal_time_us` 封装逐飞 `system_getval_us()`。

---

## 头文件与编译期检查

```c
// L1: 包含本模块声明 `motor.h`。
// L2: 全局配置（频率、舵机中位、图像尺寸宏）。
// L3: `control.h` — 调用 `control_servo_clamp` 二次限幅舵机。
// L4: 板级引脚映射 `pins.h`。
// L5: 逐飞公共头文件（`pwm_init`、`gpio_init`、`system_start` 等）。
// L6: （空行。）
// L7-L9: 编译期断言：`config.h` 的 IMG_W/IMG_H 必须与摄像头驱动 MT9V03X 分辨率一致，否则 #error。
// L10: （空行。）
```

---

## motor_hw_init

```c
// L11: 硬件初始化入口，上电调用一次。
// L12: `system_start()` — 逐飞库系统与外设基础启动。
// L13: （空行。）
// L14: 初始化舵机 PWM：引脚 PIN_SERVO_PWM，50 Hz，初占空比 SERVO_CENTER（705）。
// L15: （空行。）
// L16: 未使用的电机相关 GPIO 1：配置为推挽输出，默认低电平。
// L17: 未使用的电机相关 GPIO 2：同上。
// L18: （空行。）
// L19: 左轮 IN1：17 kHz PWM，初值 0。
// L20: 左轮 IN2：17 kHz PWM，初值 0（反向通道，正常前进保持 0）。
// L21: 右轮 IN1：17 kHz PWM，初值 0。
// L22: 右轮 IN2：17 kHz PWM，初值 0。
// L23: （空行。）
// L24: 调用 `motor_stop()`，确保上电瞬间电机无输出。
// L25: 闭括号。
```

---

## motor_apply — 正常行驶 PWM 映射

```c
// L27: 同时应用舵机与左右轮占空比（主循环正常发车路径）。
// L28: 设置舵机 PWM，经 `control_servo_clamp` 再次限幅后写入硬件。
// L29: （空行。）
// L30-L32: 若 duty 超过驱动器 `PWM_DUTY_MAX`，截断到最大值。
// L33: （空行。）
// L34: 左轮 IN2 占空比置 0（不使用反向制动）。
// L35: 右轮 IN2 占空比置 0。
// L36: 左轮 IN1 设为 duty — 左轮前进力矩。
// L37: 右轮 IN1 设为相同 duty — 左右同步前进（差速由舵机转向，非轮速差）。
// L38: 闭括号。
```

---

## motor_apply_left_only

```c
// L40: 左轮单独测试：菜单 Left Test 模式。
// L41-L43: duty 上限钳位到 PWM_DUTY_MAX。
// L44: （空行。）
// L45: 舵机强制回中位，避免测试时转向干扰。
// L46: 右轮 IN1 置 0。
// L47: 右轮 IN2 置 0。
// L48: 左轮 IN2 置 0。
// L49: 仅左轮 IN1 输出 duty。
// L50: 闭括号。
```

---

## motor_stop / motor_reset

```c
// L52: 电机完全停止（四轮 PWM 均为 0）。
// L53: 左轮 IN1 → 0。
// L54: 左轮 IN2 → 0。
// L55: 右轮 IN1 → 0。
// L56: 右轮 IN2 → 0。
// L57: 闭括号。
// L58: （空行。）
// L59: 停车并舵机回中 — 延迟阶段、超时、未 armed 时 `cpu0_main` 调用。
// L60: 先 `motor_stop()` 切断驱动力。
// L61: 舵机 PWM 设为 SERVO_CENTER（705）。
// L62: 闭括号。
```

---

## motor_apply_servo_only

```c
// L64: 仅驱动舵机，用于 Align Test / 摄像头对中调试。
// L65: 先停车，避免误转轮。
// L66: 设置舵机 PWM（限幅后）。
// L67: 闭括号。
```

---

## hal_time_us

```c
// L69: 全工程统一时间源：返回 32 位微秒时间戳。
// L70: 内部调用逐飞 `system_getval_us()` 并转为 uint32_t。
```

---

## PWM 映射关系

| 引脚角色 | 正常前进 | 停车 |
|----------|----------|------|
| PIN_SERVO_PWM | `servo_pwm`（限幅） | SERVO_CENTER 或不变 |
| LEFT/RIGHT IN1 | `duty` | 0 |
| LEFT/RIGHT IN2 | 0 | 0 |
