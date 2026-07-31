# motor.h 逐行注释

> 行号与源文件一致。

电机与舵机硬件抽象层声明：PWM 初始化、正常驱动、测试模式、停车复位，以及全工程统一的微秒时钟 `hal_time_us()`。

---

```c
// L1: 文件标识注释。
// L2: 头文件守卫 `#ifndef MOTOR_H`。
// L3: 定义 MOTOR_H。
// L4: （空行。）
// L5: 标准整数类型。
// L6: 引脚定义 `pins.h`（PIN_SERVO_PWM、PIN_MOTOR_* 等）。
// L7: 全局配置 `config.h`（SERVO_PWM_HZ、MOTOR_PWM_FREQ 等）。
// L8: （空行。）
// L9: `motor_hw_init` — 上电初始化：系统时钟、舵机/电机 PWM、GPIO、默认停车。
// L10: `motor_stop` — 四轮 PWM 全 0，电机自由停（舵机不动）。
// L11: `motor_reset` — 停车并将舵机回中位 SERVO_CENTER。
// L12: `motor_apply` — 正常行驶：同时设置舵机与左右轮等占空比前进。
// L13: `motor_apply_left_only` — 仅左轮前进，舵机回中；菜单左轮测试用。
// L14: `motor_apply_servo_only` — 停车后只转舵机；菜单对中/转向测试用。
// L15: （空行。）
// L16: `hal_time_us` — 返回自系统启动起的微秒计数，发车延迟与超时均依赖此函数（非帧计数）。
// L17: （空行。）
// L18: 结束头文件守卫 `#endif`。
```
