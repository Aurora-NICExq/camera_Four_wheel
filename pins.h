/* pins.h - pin/channel map */
#ifndef PINS_H
#define PINS_H

#define PIN_SERVO_PWM       (ATOM1_CH1_P33_9)    // 舵机信号 P33.9

/* 两路电机：各 PWM 调速 + DIR 方向 GPIO。滑行式开环永不反转 —— DIR 上电置前进电平后保持不变，
 * 两路 PWM 始终写同一占空比（无电子差速）。前进电平由 config.h 的 MOTOR_DIR_FORWARD_LEVEL 决定。 */
#define PIN_MOTOR1_PWM      (ATOM0_CH2_P21_4)    // 电机1 调速 P21.4
#define PIN_MOTOR1_DIR      (P21_5)              // 电机1 方向 P21.5
#define PIN_MOTOR2_PWM      (ATOM0_CH4_P02_4)    // 电机2 调速 P02.4
#define PIN_MOTOR2_DIR      (P02_5)              // 电机2 方向 P02.5

#define PIN_MOTOR_BRAKE     /* 未接线：ENABLE_HW_BRAKE=0 时不参与编译 */
#define PIN_BUZZER          (P33_10)             // 板载有源蜂鸣器 B1
#define IPS200_CONNECT_TYPE (IPS200_TYPE_SPI)    // 2.0" IPS200，SPI2 P15.x

/* 摄像头(MT9V03X)/无线串口/按键 引脚均由逐飞驱动头文件内部固定，此处不分配。 */

#define PINS_CONFIGURED
#ifndef PINS_CONFIGURED
#error "pins.h：请填写所有引脚后再定义 PINS_CONFIGURED。"
#endif

#endif /* PINS_H */
