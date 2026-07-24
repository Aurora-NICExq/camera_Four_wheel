/* battery.c - AN0 电池电压采样与过放保护 */
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "battery.h"

static uint8_t  s_battery_ok = 1;
static uint8_t  s_low_cnt;
static uint16_t s_check_cnt;

static uint16_t battery_adc_to_mv(uint16_t adc)
{
    uint32_t mv;

    mv = (uint32_t)adc * (uint32_t)BATTERY_ADC_REF_MV *
         (uint32_t)BATTERY_DIVIDER_NUM;
    mv /= (4095u * (uint32_t)BATTERY_DIVIDER_DEN);
    return (uint16_t)mv;
}

void battery_init(void)
{
    adc_init(PIN_BATTERY_ADC, BATTERY_ADC_RESOLUTION);
    s_battery_ok = 1;
    s_low_cnt = 0;
    s_check_cnt = 0;
}

void battery_update(void)
{
    uint16_t raw;
    uint16_t mv;

    if (s_check_cnt < BATTERY_CHECK_PERIOD_FRAMES)
    {
        s_check_cnt++;
        return;
    }
    s_check_cnt = 0;

    raw = adc_mean_filter_convert(PIN_BATTERY_ADC, BATTERY_ADC_SAMPLES);
    mv = battery_adc_to_mv(raw);

    if (!s_battery_ok)
    {
        return;
    }

    if (mv < BATTERY_LOW_THRESH_MV)
    {
        if (s_low_cnt < BATTERY_LOW_FRAMES)
        {
            s_low_cnt++;
        }
        if (s_low_cnt >= BATTERY_LOW_FRAMES)
        {
            s_battery_ok = 0;
        }
    }
    else
    {
        s_low_cnt = 0;
    }
}

uint8_t battery_ok(void)
{
    return s_battery_ok;
}
