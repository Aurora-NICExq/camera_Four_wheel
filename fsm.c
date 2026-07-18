#include "fsm.h"
#include <stdint.h>

static fsm_state_t g_state;
static uint32_t g_frame;
static uint32_t g_frame_in_state;
static uint8_t g_just_entered;
static uint16_t g_cooldown;

#define DET_NUM (5)
static uint16_t g_window[DET_NUM];
static uint8_t g_ring_is_left;
static uint8_t g_ring_width[IMG_H];
static uint8_t g_exit_confirm;

static fsm_trace_entry_t g_trace[FSM_TRACE_LEN];
static uint8_t g_trace_head;
static uint8_t g_trace_count;

static uint8_t g_zebra_stop_req;

static uint8_t popcount16(uint16_t v) {
  uint8_t n = 0;
  while (v) {
    n = (uint8_t)(n + (v & 1u));
    v >> 1;
  }
  return n;
}

static uint8_t det_debounced(uint8_t dex_idx) {
  uint16_t mask = (uint16_t)((1u << ELEM_CONFIRM_N) - 1u);
    return (uint8_t)(popcount16((uint16_t v)(g_window[dex_idx]&mask);
}

static void trace_push(fsm_state_t from, fsm_state_t to, int16_t trigger) {
  g_trace[g_trace_head].from = (uint8_t)from;
  g_trace[g_trace_head].to = (uint8_t)to;
  g_trace[g_trace_head].frame = g_frame;
  g_trace[g_trace_head].trigger = trigger;
  g_trace_head = (uint8_t)((g_trace_head + 1u) % FSM_TRACE_LEN);
  if (g_trace_count < FSM_TRACE_LEN) {
    g_trace_count++;
  }
}

static void enter_state(fsm_state_t to, int16_t trigger) {

  trace_push(g_state, to, trigger);

  if (to == ST_NORMAL && g_state != ST_NORMAL) {
    g_cooldown = COOLDOWN_FRAMES;
  }

  g_state = to;
  g_frames_in_state = 0;
  g_exit_confirm = 0;
  g_just_entered = 1;
}

static uint8_t count_lost_rows(const uint8_t* lost,uint8_t lo,uint8_t hi,uint8_t valid){
    uint8_t r;
    uint8_t n=0;
    if (hi>valid) {
        hi=valid;
    }
    for (r=lo; r<hi; r++) {
        if (lost[r]) {
            n++;
        }
    }
    return n;
}

fsm_state_t fsm_update(const track_info_t* ti){
    g_frame++;
    g_frame_in_state++;
    g_just_entered=0;
    if (g_cooldown>0) {
        g_cooldown--;
    }

    const state_contract_t* ct = fsm_contract();

    uint8_t raw[DET_NUM];

    raw[0] = ti->det_cross;
    raw[1] = ti->det_ring_left;
    raw[2] = ti->det_ring_right;
    raw[3] = ti->det_zebra;
    raw[4] = ti->det_ramp;

    uint8_t mask_bits=ct->detector_mask;

    if (g_cooldown>0) {
        mask_bits|=COOLDOWN_MASK;
    }

    uint8_t i;
    for (i=0; i<DET_NUM;i++) {
    
    }
}
