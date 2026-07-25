/* motor.h */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "pins.h"
#include "config.h"

void motor_hw_init(void);
void motor_stop(void);
void motor_reset(void);
void motor_apply(uint16_t servo_pwm, uint16_t duty);
void motor_apply_servo_only(uint16_t servo_pwm);

#endif
