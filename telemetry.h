/* telemetry.h — 跑车数据记录:整圈存 RAM,停车后经调试串口吐 CSV
 *
 * 用法(接线见 telemetry.c 顶部):
 *   1. 菜单 Armed ON,从这一刻就开始记(含发车延时那 2 秒)
 *   2. 跑完(Stop Time 到 / 丢线保护 / 手动撤 Armed)后把 Armed 关掉
 *   3. 菜单进 Dump Log,串口助手抓下来存成 .csv,直接丢 Excel
 *
 * 注意:再次 Armed ON 会清空上一趟的数据。要留就先 Dump。
 *
 * 记录期间不发一个字节,只往数组里写 16 字节 —— 不占控制周期,
 * 也不受串口链路阻塞影响。
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "control.h"
#include "image.h"

/* 开始新一次记录:清空计数,重置时间原点。Armed 起始那一帧调一次。 */
void telemetry_start(void);

/* 记一帧。时间戳由本模块用 hal_time_us() 自己算,不吃帧计数 ——
   主循环会因图像处理/刷屏掉帧,帧号乘以 20ms 得到的时间轴是错的。 */
void telemetry_log(const track_info_t *ti, const control_out_t *out,
                   uint8_t drive_en);

/* 阻塞把整段数据打成 CSV 发出去。车必须是停的(菜单里已挡住 Armed 态)。 */
void telemetry_dump(void);

uint16_t telemetry_count(void);    /* 已记录帧数 */
uint8_t  telemetry_overflow(void); /* 1 = 缓冲写满,数据被截断 */

#endif /* TELEMETRY_H */
