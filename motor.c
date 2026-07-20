/* motor.c - servo/motor PWM HAL */
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "control.h"    /* control_servo_clamp：舵机第二道（最后一道）机械限位钳制 */
#include "motor.h"

/* 逻辑层图像尺寸必须与摄像头驱动一致 —— 不一致就在这里编译报错，别等上车再发现 */
#if (IMG_W != MT9V03X_W) || (IMG_H != MT9V03X_H)
#error "config.h 的 IMG_W/IMG_H 必须与 zf_device_mt9v03x.h 的 MT9V03X_W/MT9V03X_H 一致"
#endif

void motor_hw_init(void)
{
    system_start();                                             /* STM 定时器：hal_time_us() 时基     */

    pwm_init(PIN_SERVO_PWM,  SERVO_PWM_HZ, SERVO_CENTER);       /* 舵机：50 Hz，上电即中位            */
    pwm_init(PIN_MOTOR1_PWM, MOTOR_PWM_HZ, 0);                  /* 电机1：~17 kHz，上电即 0           */
    pwm_init(PIN_MOTOR2_PWM, MOTOR_PWM_HZ, 0);                  /* 电机2：~17 kHz，上电即 0           */

    /* 方向 GPIO：上电即前进电平（MOTOR_DIR_FORWARD_LEVEL=0 → 低电平正转）。
     * 本车滑行式开环、永不反转 —— 两路 DIR 初始化后保持不变，两路 PWM 始终写入同一占空比。 */
    gpio_init(PIN_MOTOR1_DIR, GPO,
              MOTOR_DIR_FORWARD_LEVEL ? GPIO_HIGH : GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(PIN_MOTOR2_DIR, GPO,
              MOTOR_DIR_FORWARD_LEVEL ? GPIO_HIGH : GPIO_LOW, GPO_PUSH_PULL);

#if ENABLE_HW_BRAKE
    /* 驱动板刹车脚（实验挂钩，默认不编译；任何减速逻辑不得依赖它） */
    gpio_init(PIN_MOTOR_BRAKE, GPO, GPIO_LOW, GPO_PUSH_PULL);
#endif

    gpio_init(PIN_BUZZER, GPO, GPIO_LOW, GPO_PUSH_PULL);        /* 有源蜂鸣器，低电平静音             */

    key_init(1000 / FRAMES_PER_SECOND);                         /* 按键扫描周期 = 帧周期（ms），
                                                                   key_scanner 由主循环每帧调用一次   */
}

void motor_apply(uint16_t servo_pwm, uint16_t duty)
{
    pwm_set_duty(PIN_SERVO_PWM, control_servo_clamp((int32_t)servo_pwm));

    if (duty > PWM_DUTY_MAX)
    {
        duty = PWM_DUTY_MAX;
    }
    pwm_set_duty(PIN_MOTOR1_PWM, duty);                        /* 两路并联同速：无电子差速 */
    pwm_set_duty(PIN_MOTOR2_PWM, duty);
}

void motor_stop(void)
{
    pwm_set_duty(PIN_MOTOR1_PWM, 0);
    pwm_set_duty(PIN_MOTOR2_PWM, 0);
}

uint32_t hal_time_us(void)
{
    return (uint32_t)system_getval_us();
}

/* 按键 / 蜂鸣器薄封装 */
void hal_key_scan(void)
{
    key_scanner();
}

uint8_t hal_key_pressed(uint8_t key_index)
{
    if (key_index >= KEY_NUMBER)
    {
        return 0;
    }
    if (key_get_state((key_index_enum)key_index) == KEY_SHORT_PRESS)
    {
        key_clear_state((key_index_enum)key_index);     /* 读取即消费：一次按下只触发一次 */
        return 1;
    }
    return 0;
}

void hal_buzzer_on(void)
{
    gpio_set_level(PIN_BUZZER, GPIO_HIGH);
}

void hal_buzzer_off(void)
{
    gpio_set_level(PIN_BUZZER, GPIO_LOW);
}
