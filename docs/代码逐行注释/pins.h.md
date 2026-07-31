# pins.h 逐行注释

> 行号与源文件一致。

板级引脚与外设通道宏定义：舵机 PWM、左右电机四路 PWM、IPS200 背光、四键 GPIO、SPI 屏类型。改线时只改本文件，业务代码通过宏名引用。

```c
// L1: 文件头注释
/* pins.h */
// L2: 头文件保护开始
#ifndef PINS_H
// L3: 定义 PINS_H 宏
#define PINS_H
// L4: 空行

// L5: 舵机 PWM：ATOM1 通道 4，引脚 P21_6
#define PIN_SERVO_PWM       (ATOM1_CH4_P21_6)
// L6: 空行

// L7: 左电机正转 PWM：ATOM0 CH2 P21_4
#define PIN_MOTOR_LEFT_IN1  (ATOM0_CH2_P21_4)
// L8: 左电机反转 PWM：ATOM0 CH3 P21_5
#define PIN_MOTOR_LEFT_IN2  (ATOM0_CH3_P21_5)
// L9: 右电机正转 PWM：ATOM0 CH1 P21_3
#define PIN_MOTOR_RIGHT_IN1 (ATOM0_CH1_P21_3)
// L10: 右电机反转 PWM：ATOM0 CH0 P21_2
#define PIN_MOTOR_RIGHT_IN2 (ATOM0_CH0_P21_2)
// L11: 空行

// L12: 预留 GPIO P02_6，当前未接电机
#define PIN_MOTOR_UNUSED_1  (P02_6)
// L13: 预留 GPIO P02_7
#define PIN_MOTOR_UNUSED_2  (P02_7)
// L14: 空行

// L15: 刹车引脚占位；本板未使用，注释掉无实际宏
#define PIN_MOTOR_BRAKE     /* unused */
// L16: 空行

// L17: IPS200 背光说明：逐飞默认 P15_4，本板改 P20_14
/* IPS200 背光：逐飞库默认 P15_4，本板改接 P20_14（须在 ips200_init 后再次 gpio_init） */
// L18: 背光控制 GPIO：P20_14，menu_port_init 中拉高点亮
#define PIN_IPS200_BL       (P20_14)
// L19: 空行

// L20: 四键均为上拉输入，按下读低电平（见 menu_port.c KEY_ACTIVE_LEVEL）
/* 四个独立按键（上拉输入，按下为低电平） */
// L21: KEY1 上键：P13_3
#define PIN_KEY_UP          (P13_3)   /* KEY1 UP */
// L22: KEY2 下键：P11_9
#define PIN_KEY_DOWN        (P11_9)   /* KEY2 DOWN */
// L23: KEY3 确认：P11_10（丝印可能标 RIGHT）
#define PIN_KEY_ENTER       (P11_10)  /* KEY3 RIGHT */
// L24: KEY4 返回：P11_11
#define PIN_KEY_BACK        (P11_11)  /* KEY4 BACK */
// L25: 空行

// L26: IPS200 使用 SPI 接口（非并口）
#define IPS200_CONNECT_TYPE (IPS200_TYPE_SPI)
// L27: 空行

// L28: 结束头文件保护
#endif /* PINS_H */
```
