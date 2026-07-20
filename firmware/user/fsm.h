/*********************************************************************************************************************
 * 模块：fsm.h — 唯一状态机（纯逻辑层）
 *
 * 约束：不包含任何 MCU / 逐飞库头文件，只允许 <stdint.h>、config.h、image.h。
 *
 * 精简版（直道+转弯测试赛道）：已移除十字 / 环岛状态与检测器。
 * 保留 NORMAL 巡线，以及坡道占位、超时恢复、故障吸收态。
 ********************************************************************************************************************/
#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include "config.h"
#include "image.h"

/*---- 状态枚举：新增元素状态必须同时更新 fsm.c 的 switch 与契约表（编译期断言保证同长） ----*/
typedef enum
{
    ST_NORMAL = 0,      /* 正常巡线：加权中线转向                                   */
    ST_RAMP,            /* 坡道（占位）：限速直行                                 */
    ST_RECOVERY,        /* 元素超时后的保守寻线；健康确认后回 NORMAL               */
    ST_FAULT,           /* 恢复失败吸收态：断油并请求主流程解除武装                 */
    ST_COUNT            /* 状态总数（仅用于表长度与编译期检查，不是状态）          */
} fsm_state_t;

/*---- 转向源：契约表告诉 control.c 本状态用什么几何量生成误差 ----*/
typedef enum
{
    STEER_SRC_MIDLINE = 0,  /* 加权中线误差（track_info_t.error）            */
} steer_src_t;

/*---- 状态契约：control.c 每帧读一行，除此之外不感知状态语义 ----*/
typedef struct
{
    steer_src_t steer_src;      /* 转向误差来源                                      */
    int16_t     steer_bias;     /* 保留字段（当前恒为 0）                            */
    uint16_t    duty_cap;       /* 本状态占空比上限 [0,10000]                         */
    uint8_t     detector_mask;  /* 本状态内被屏蔽的检测器位掩码（DET_BIT_x 组合）     */
} state_contract_t;

/*---- 检测器位（去抖计数、冷却掩码、契约表屏蔽共用同一套位定义） ----*/
#define DET_BIT_RAMP        (1u << 0)

/*---- 转移轨迹：最近 16 次状态转移的环形缓冲，屏幕第 2 页 + 串口 dump 用 ----*/
typedef struct
{
    uint8_t  from;          /* 源状态（fsm_state_t）                    */
    uint8_t  to;            /* 目标状态                                 */
    uint32_t frame;         /* 发生转移的帧号                           */
    int16_t  trigger;       /* 触发值（det_value / 超时记 -1）          */
} fsm_trace_entry_t;

#define FSM_TRACE_LEN   16  /* 环形缓冲深度（2 的幂，便于取模）          */

void fsm_init(void);
fsm_state_t fsm_update(const track_info_t *ti);

fsm_state_t             fsm_state(void);
const state_contract_t *fsm_contract(void);
uint8_t                 fsm_state_just_entered(void);
const fsm_trace_entry_t*fsm_trace(uint8_t idx_back);
uint32_t                fsm_frame_count(void);
uint8_t                 fsm_fault_request(void);

#endif /* FSM_H */
