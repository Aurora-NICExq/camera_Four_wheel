/*********************************************************************************************************************
 * 模块：perf.h — 流水线分段耗时测量的挂钩（纯逻辑层，PC/MCU 双端共用）
 *
 * 约束：本头文件只含 <stdint.h> 与 config.h —— image.c 依赖它，必须保持 PC (gcc) 可编译。
 *
 * 原理：PERF_PROFILE=0（默认）时所有宏展开为 ((void)0)，编译产物与不含本文件时逐位一致 ——
 *       挂钩本身不构成任何行为差异，replay 金标准输出不变。
 *       PERF_PROFILE=1 时由使用侧提供三个函数的实现：
 *         - 车上：user/perf.c（hal_time_us() 计时，统计 min/mean/max/p99，无线串口定期输出）
 *         - PC ：test/profile.c（clock_gettime 计时，离线热点分析）
 *       同一阶段一帧内允许多次 begin/end（自动累加），每帧结束由主循环调用 PERF_COMMIT() 结算。
 *
 * 为什么用"外部函数 + 宏"而不是直接在 image.c 里调 hal_time_us()：
 *       image.c 禁止包含任何 MCU 头文件（PC 回放逐位一致的前提）；时间源由链接期注入。
 ********************************************************************************************************************/
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
