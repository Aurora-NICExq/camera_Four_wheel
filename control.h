/*********************************************************************************************************************
 * 模块：control.h — 转向 PD + 定速开环占空比（纯逻辑层）
 *
 * 约束：不包含任何 MCU / 逐飞库头文件，只允许 <stdint.h>、config.h、image.h、fsm.h。
 *
 * 精简版：速度不再做行数/曲率/转向/boost 调度，仅输出 STRAIGHT_DUTY（受契约与硬上限约束）并斜坡。
 ********************************************************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "config.h"
#include "image.h"
#include "fsm.h"

typedef struct
{
    uint16_t servo_pwm;
    uint16_t duty;
    int16_t  error_used;
    uint16_t duty_target;
} control_out_t;

void control_init(void);
void control_update(const track_info_t *ti, uint8_t armed, control_out_t *out);
uint16_t control_servo_clamp(int32_t servo_raw);

#endif /* CONTROL_H */
