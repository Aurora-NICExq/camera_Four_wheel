/* pins.h - pin/channel map */
#ifndef PINS_H
#define PINS_H

#define PIN_SERVO_PWM       (ATOM1_CH4_P21_6)

#define PIN_MOTOR_LEFT_IN1  (ATOM0_CH2_P21_4)
#define PIN_MOTOR_LEFT_IN2  (ATOM0_CH3_P21_5)
#define PIN_MOTOR_RIGHT_IN1 (ATOM0_CH1_P21_3)
#define PIN_MOTOR_RIGHT_IN2 (ATOM0_CH0_P21_2)

#define PIN_MOTOR_UNUSED_1  (P02_6)
#define PIN_MOTOR_UNUSED_2  (P02_7)

#define PIN_MOTOR_BRAKE     /* unused */
#define PIN_BUZZER          (P33_10)

/* IPS200 背光：逐飞库默认 P15_4，本板改接 P20_14（须在 ips200_init 后再次 gpio_init） */
#define PIN_IPS200_BL       (P20_14)

/* 逐飞主板电池电压检测：AN0 → ADC0 通道 0 */
#define PIN_BATTERY_ADC     (ADC0_CH0_A0)

/* 逐飞主板四个独立按键 KEY1~KEY4（上拉输入，按下为低电平） */
#define PIN_KEY_UP          (P20_6)   /* KEY1 */
#define PIN_KEY_DOWN        (P20_7)   /* KEY2 */
#define PIN_KEY_ENTER       (P11_2)   /* KEY3 */
#define PIN_KEY_BACK        (P11_3)   /* KEY4 */

#define IPS200_CONNECT_TYPE (IPS200_TYPE_SPI)

#endif /* PINS_H */
