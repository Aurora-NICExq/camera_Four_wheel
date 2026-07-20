/* motor.h - motor/servo/buzzer HAL API */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_hw_init(void);

void motor_apply(uint16_t servo_pwm, uint16_t duty);

void motor_stop(void);

uint32_t hal_time_us(void);

uint8_t hal_key_pressed(uint8_t key_index);     /* 1 = 该键本次扫描为短按（读取即清除） */
void    hal_key_scan(void);                     /* 每帧调用一次的按键扫描               */
void    hal_buzzer_on(void);                    /* 蜂鸣器开（chirp 时长由上层帧计数控制）*/
void    hal_buzzer_off(void);

#endif /* MOTOR_H */
