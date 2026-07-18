#ifndef FSM_H
#define FSM_H

#include "config.h"
#include "image.h"
#include <stdint.h>
typedef enum {
  ST_NORMAL = 0,
  ST_CROSS,
  ST_RING_PRE,
  ST_RING_IN,
  ST_RING,
  EXIT,
  ST_ZEBRA,
  ST_RAMP,
  ST_COUNT,
} fsm_state_t;

typedef enum {
  STEER_SRC_MIDLINE = 0,
  STEER_SRC_LEFT_EDGE,
  STEER_SRC_RIGHT_EDGE,
  STEER_SRC_FIXED_BIAS,
} steer_src_t;

typedef struct {
  steer_src_t steer_src; // 转向误差
  int16_t steer_bias;
  uint16_t duty_cap; // 占空比
  uint8_t detector_mask;
} state_contract_t;

#define DET_BIT_CROSS (1u << 0)
#define DET_BIT_RING_LEFT (1u << 1)
#define DET_BIT_RING_RIGHT (1u << 2)
#define DET_BIT_ZEBRA (1u << 3)
#define DET_BIT_RAMP (1u << 4)

typedef struct {
  uint8_t from;
  uint8_t to;
  uint32_t frame;
  int16_t trigger;
} fsm_trace_entry_t;

#define FSM_TRACE_LEN 16

void fsm_init(void);
fsm_state_t fsm_update(const track_info_t *ti);
const state_contract_t *fsm_contract(void);
uint8_t fsm_state_just_entered(void);
uint16_t fsm_ring_half_width(uint8_t row);
const fsm_trace_entry_t *fsm_trace(uint8_t idx_back);
uint32_t fsm_frame_count(void);
uint8_t fsm_zebra_stop_request(void);
uint8_t fsm_ring_is_left(void);

#endif
