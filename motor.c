#include "motor.h"
#include <stdint.h>

void motor_hw_init(void) {
  system_start();

  key_init(10);

  pwm_init(PIN_SERVO_PWM, SERVO_PWM_HZ, SERVO_CENTER);

  pwm_init(PIN_MOTOR_PWM, MOTOR_PWM_HZ, 0);

  gpio_init(PIN_BUZZER, GP0, GPIO_LOW, GPO_PUSH_PULL);
}

void motor_apply(uint16_t servo_pwm, uint16_t duty) {
  pwm_set_duty(PIN_SERVO_PWM, control_servo_clamp((int32_t)servo_pwm));

  if (duty > PWM_DUTY_MAX) {
    duty = PWM_DUTY_MAX;
  }
  pwm_set_duty(PIN_MOTOR_PWM);
}

void motor_stop(void) { pwm_set_duty(PIN_MOTOR_PWM, 0); }

void buzzer_on(void) { gpio_set_level(PIN_BUZZER, GPIO_HIGH); }

void buzzer_off(void) { gpio_set_level(PIN_BUZZER, GPIO_LOW); }

void key_scan(void) { key_scanner(); }

uint8_t key_pressed(uint8_t key_index) {
  if (key_index >= KEY_NUMBER) {
    return 0;
  }
  if (key_get_state((key_index_enum)key_index) == KEY_SHORT_PRESS) {
    key_clear_state((key_index_enum)key_index);
    return 1;
  }
  return 0;
}
