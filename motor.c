/* motor.c - servo/motor PWM HAL (4-PWM H-bridge) */
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "control.h"
#include "motor.h"

#if (IMG_W != MT9V03X_W) || (IMG_H != MT9V03X_H)
#error "config.h IMG_W/IMG_H must match MT9V03X_W/MT9V03X_H"
#endif

void motor_hw_init(void)
{
    system_start();

    pwm_init(PIN_SERVO_PWM, SERVO_PWM_HZ, SERVO_CENTER);

    gpio_init(PIN_MOTOR_UNUSED_1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(PIN_MOTOR_UNUSED_2, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(PIN_MOTOR_LEFT_IN1, MOTOR_PWM_FREQ, 0U);
    pwm_init(PIN_MOTOR_LEFT_IN2, MOTOR_PWM_FREQ, 0U);
    pwm_init(PIN_MOTOR_RIGHT_IN1, MOTOR_PWM_FREQ, 0U);
    pwm_init(PIN_MOTOR_RIGHT_IN2, MOTOR_PWM_FREQ, 0U);

    motor_stop();
}

void motor_apply(uint16_t servo_pwm, uint16_t duty)
{
    pwm_set_duty(PIN_SERVO_PWM, control_servo_clamp((int32_t)servo_pwm));

    if (duty > PWM_DUTY_MAX) { duty = PWM_DUTY_MAX; }

    pwm_set_duty(PIN_MOTOR_LEFT_IN2, 0U);
    pwm_set_duty(PIN_MOTOR_RIGHT_IN2, 0U);
    pwm_set_duty(PIN_MOTOR_LEFT_IN1, duty);
    pwm_set_duty(PIN_MOTOR_RIGHT_IN1, duty);
}

void motor_stop(void)
{
    pwm_set_duty(PIN_MOTOR_LEFT_IN1, 0U);
    pwm_set_duty(PIN_MOTOR_LEFT_IN2, 0U);
    pwm_set_duty(PIN_MOTOR_RIGHT_IN1, 0U);
    pwm_set_duty(PIN_MOTOR_RIGHT_IN2, 0U);
}

void motor_reset(void)
{
    motor_stop();
    pwm_set_duty(PIN_SERVO_PWM, SERVO_CENTER);
}

void motor_apply_servo_only(uint16_t servo_pwm)
{
    motor_stop();
    pwm_set_duty(PIN_SERVO_PWM, control_servo_clamp((int32_t)servo_pwm));
}
