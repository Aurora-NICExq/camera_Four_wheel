/* perf.h - PERF_BEGIN/END stage hooks */
#ifndef PERF_H
#define PERF_H

#include <stdint.h>
#include "config.h"     /* PERF_PROFILE 开关 */

/* 阶段编号：报表顺序即流水线顺序。PF_TOTAL 与 cpu0_main.c 的 proc_us 覆盖同一区间。 */
enum
{
    PF_OTSU = 0,        /* 二值化阈值（大津直方图 + 阈值扫描）                    */
    PF_LONGCOL,         /* 最长白列                                              */
    PF_EDGES,           /* 逐行边线搜索                                          */
    PF_REBUILD,         /* 丢线重建 + 中线                                       */
    PF_ERRCURV,         /* 加权转向误差 + 曲率                                   */
    PF_DET_GEOM,        /* 几何检测器：十字 / 环岛 / 坡道（只读边线数组）        */
    PF_FSM,             /* fsm_update                                            */
    PF_CONTROL,         /* control_update                                        */
    PF_DISPLAY,         /* display_update + display_telemetry（在 proc_us 之外） */
    PF_TOTAL,           /* 主循环 t0→proc_us 区间（图像+状态机+按键+控制+电机）  */
    PF_STAGE_COUNT
};

#if PERF_PROFILE

void perf_stage_begin(uint8_t stage_id);    /* 记录该阶段本次起点时间戳                     */
void perf_stage_end(uint8_t stage_id);      /* 累加本次耗时到该阶段的"本帧暂存"             */
void perf_frame_commit(void);               /* 每帧一次：本帧各阶段暂存 → 统计量，清零暂存  */

#define PERF_BEGIN(s)   perf_stage_begin(s)
#define PERF_END(s)     perf_stage_end(s)
#define PERF_COMMIT()   perf_frame_commit()

#else

#define PERF_BEGIN(s)   ((void)0)
#define PERF_END(s)     ((void)0)
#define PERF_COMMIT()   ((void)0)

#endif /* PERF_PROFILE */

#endif /* PERF_H */
