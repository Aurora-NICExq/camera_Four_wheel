/* pins.h - pin/channel map */
#ifndef PINS_H
#define PINS_H

#define PIN_SERVO_PWM       (ATOM1_CH1_P33_9)

#define PIN_MOTOR_LEFT_IN1  (ATOM0_CH2_P21_4)
#define PIN_MOTOR_LEFT_IN2  (ATOM0_CH3_P21_5)
#define PIN_MOTOR_RIGHT_IN1 (ATOM0_CH1_P21_3)
#define PIN_MOTOR_RIGHT_IN2 (ATOM0_CH0_P21_2)

#define PIN_MOTOR_UNUSED_1  (P02_6)
#define PIN_MOTOR_UNUSED_2  (P02_7)

#define PIN_MOTOR_BRAKE     /* unused */
#define PIN_BUZZER          (P33_10)
#define IPS200_CONNECT_TYPE (IPS200_TYPE_SPI)

#endif /* PINS_H */
