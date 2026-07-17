#ifndef _motor_h
#define _motor_h

#include <cstdint>
void motor_hw_init(void);
void motor_apply(uint16_t servo_pwm, uint16_t duty);
void motor_stop(void);
uint32_t time_us(void);
void key_scan(void);
uint8_t key_pressed(uint8_t key_index);
void buzzer_on(void);
void buzzer_off(void);

#endif
