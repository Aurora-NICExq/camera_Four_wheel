/* fsm.h - FSM states, contracts, detectors */
#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include "config.h"
#include "image.h"

/* 状态枚举 */
typedef enum
{
    ST_NORMAL = 0,      /* 正常巡线：加权中线转向，速度由减速表全权决定           */
    ST_CROSS,           /* 十字通过：拐点连线补线，直行冲过                       */
    ST_RING_PRE,        /* 环岛预备：已确认环岛，等待入环口（单边巡线，减速）     */
    ST_RING_IN,         /* 环内：单边 ± 入环前学习的半宽巡线                      */
    ST_RING_EXIT,       /* 出环：固定偏置打舵驶出，直到双边恢复                   */
    ST_RAMP,            /* 坡道（占位）：限速直行                                 */
    ST_RECOVERY,        /* 元素超时后的保守寻线；健康确认后回 NORMAL               */
    ST_FAULT,           /* 恢复失败吸收态：断油并请求主流程解除武装                 */
    ST_COUNT            /* 状态总数（仅用于表长度与编译期检查，不是状态）          */
} fsm_state_t;

/* 转向源 */
typedef enum
{
    STEER_SRC_MIDLINE = 0,  /* 加权中线误差（track_info_t.error）            */
    STEER_SRC_LEFT_EDGE,    /* 左边线 + 半宽 重建中线（环内右伪边时用）       */
    STEER_SRC_RIGHT_EDGE,   /* 右边线 - 半宽 重建中线                        */
    STEER_SRC_FIXED_BIAS,   /* 固定偏置（出环导向），偏置值在契约表中给出     */
} steer_src_t;

/* 状态契约 */
typedef struct
{
    steer_src_t steer_src;      /* 转向误差来源                                      */
    int16_t     steer_bias;     /* STEER_SRC_FIXED_BIAS 时的固定误差（像素）；其余 0  */
    uint16_t    duty_cap;       /* 本状态占空比上限 [0,10000]，与减速表取 min        */
    uint8_t     detector_mask;  /* 本状态内被屏蔽的检测器位掩码（DET_BIT_x 组合）     */
} state_contract_t;

/* 检测器位（去抖计数、冷却掩码、契约表屏蔽共用同一套位定义） */
#define DET_BIT_CROSS       (1u << 0)
#define DET_BIT_RING_LEFT   (1u << 1)
#define DET_BIT_RING_RIGHT  (1u << 2)
#define DET_BIT_RAMP        (1u << 3)

/* 转移轨迹 */
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

fsm_state_t             fsm_state(void);                    /* 当前状态                                     */
const state_contract_t *fsm_contract(void);                 /* 当前状态的契约行                             */
uint8_t                 fsm_state_just_entered(void);       /* 1 = 本帧刚发生状态切换（control 用于 D 项重载）*/
uint16_t                fsm_ring_half_width(uint8_t row);   /* 入环前学习的逐行半宽快照（环内单边巡线用）    */
const fsm_trace_entry_t*fsm_trace(uint8_t idx_back);        /* 取最近第 idx_back 条转移记录（0=最新，NULL=无）*/
uint32_t                fsm_frame_count(void);              /* 状态机累计帧号（所有超时/冷却的时基）         */
uint8_t                 fsm_fault_request(void);            /* 1 = 元素超时后恢复失败，请求解除武装           */
uint8_t                 fsm_ring_is_left(void);             /* 1 = 当前环岛序列为左环（control 取偏置符号用） */

#endif /* FSM_H */
