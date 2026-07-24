/* battery.h - 电池电压 ADC 与过放保护 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

void     battery_init(void);
void     battery_update(void);
uint8_t  battery_ok(void);

#endif /* BATTERY_H */
