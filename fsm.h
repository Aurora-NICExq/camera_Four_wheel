#ifndef _FSM_H__
#define _FSM_H__

#include <cstdint>
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
}

#endif
