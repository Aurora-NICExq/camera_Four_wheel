/*********************************************************************************************************************
 * 模块：fsm.c — 唯一状态机（纯逻辑层，可在 PC 上用 gcc 编译）
 *
 * 精简版状态图（直道 + 转弯）：
 *
 *                       ┌───────────────────────────────────────────┐
 *                       │                 ST_NORMAL                 │
 *                       │ 去抖 M/N + 冷却掩码（坡道占位）            │
 *                       └──────────────────┬────────────────────────┘
 *                                 坡道去抖 │
 *                                          ▼
 *                                   ┌──────────┐
 *                                   │ ST_RAMP  │ ──保持帧数满──► NORMAL(冷却)
 *                                   └──────────┘
 *
 *   ※ 元素超时 → ST_RECOVERY → 失败则 ST_FAULT。
 *   ※ 失控保护不在本状态机内（见 cpu0_main.c）。
 ********************************************************************************************************************/
#include "fsm.h"
#include "config.h"
#include "image.h"
#include <stdint.h>

#define FSM_STATIC_ASSERT(cond, tag) typedef char fsm_static_assert_##tag[(cond) ? 1 : -1]

/*===================================================================================================================
 * 一、状态契约表
 *==================================================================================================================*/

static const state_contract_t g_contract_base[] = {
    /* ST_NORMAL    */ {STEER_SRC_MIDLINE, 0, DUTY_HARD_CAP, 0},
    /* ST_RAMP      */ {STEER_SRC_MIDLINE, 0, RAMP_DUTY_CAP, DET_BIT_RAMP},
    /* ST_RECOVERY  */ {STEER_SRC_MIDLINE, 0, RECOVERY_DUTY_CAP, DET_BIT_RAMP},
    /* ST_FAULT     */ {STEER_SRC_MIDLINE, 0, 0, DET_BIT_RAMP},
};

FSM_STATIC_ASSERT(sizeof(g_contract_base) / sizeof(g_contract_base[0]) == ST_COUNT,
                  contract_base_covers_all_states);
FSM_STATIC_ASSERT(RAMP_CONFIRM_N >= 1 && RAMP_CONFIRM_N <= 16 &&
                      RAMP_CONFIRM_M >= 1 && RAMP_CONFIRM_M <= RAMP_CONFIRM_N,
                  ramp_debounce_config_valid);

/*===================================================================================================================
 * 二、内部状态
 *==================================================================================================================*/

static fsm_state_t g_state;
static uint32_t g_frame;
static uint32_t g_frames_in_state;
static uint8_t g_just_entered;
static uint16_t g_cooldown;

#define DET_NUM (1)
static uint16_t g_window[DET_NUM];

static uint8_t g_exit_confirm;

static fsm_trace_entry_t g_trace[FSM_TRACE_LEN];
static uint8_t g_trace_head;
static uint8_t g_trace_count;

static uint8_t g_fault_req;

/*===================================================================================================================
 * 三、内部工具
 *==================================================================================================================*/

static uint8_t popcount16(uint16_t v) {
  uint8_t n = 0;
  while (v) {
    n = (uint8_t)(n + (v & 1u));
    v >>= 1;
  }
  return n;
}

static uint8_t det_debounced(uint8_t det_idx, uint8_t confirm_m,
                             uint8_t confirm_n) {
  uint16_t mask = (confirm_n == 16u)
                      ? 0xFFFFu
                      : (uint16_t)((1u << confirm_n) - 1u);
  return (uint8_t)(popcount16((uint16_t)(g_window[det_idx] & mask)) >=
                   confirm_m);
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
  if (to == ST_FAULT) {
    g_fault_req = 1;
  }
  g_just_entered = 1;
}

/*===================================================================================================================
 * 四、对外接口
 *==================================================================================================================*/

void fsm_init(void) {
  uint8_t i;
  g_state = ST_NORMAL;
  g_frame = 0;
  g_frames_in_state = 0;
  g_just_entered = 0;
  g_cooldown = 0;
  g_exit_confirm = 0;
  g_trace_head = 0;
  g_trace_count = 0;
  g_fault_req = 0;
  for (i = 0; i < DET_NUM; i++) {
    g_window[i] = 0;
  }
}

fsm_state_t fsm_state(void) { return g_state; }

uint8_t fsm_state_just_entered(void) { return g_just_entered; }

uint8_t fsm_fault_request(void) { return g_fault_req; }

uint32_t fsm_frame_count(void) { return g_frame; }

const state_contract_t *fsm_contract(void) {
  if ((uint8_t)g_state >= (uint8_t)ST_COUNT) {
    return &g_contract_base[ST_FAULT];
  }
  return &g_contract_base[g_state];
}

const fsm_trace_entry_t *fsm_trace(uint8_t idx_back) {
  if (idx_back >= g_trace_count) {
    return (const fsm_trace_entry_t *)0;
  }
  uint8_t idx =
      (uint8_t)((g_trace_head + FSM_TRACE_LEN - 1u - idx_back) % FSM_TRACE_LEN);
  return &g_trace[idx];
}

fsm_state_t fsm_update(const track_info_t *ti) {
  g_frame++;
  g_frames_in_state++;
  g_just_entered = 0;
  if (g_cooldown > 0) {
    g_cooldown--;
  }

  const state_contract_t *ct = fsm_contract();
  uint8_t raw[DET_NUM];
  raw[0] = ti->det_ramp;

  uint8_t mask_bits = ct->detector_mask;
  if (g_cooldown > 0) {
    mask_bits |= COOLDOWN_MASK;
  }

  uint8_t i;
  for (i = 0; i < DET_NUM; i++) {
    uint8_t hit = raw[i];
    if (mask_bits & (uint8_t)(1u << i)) {
      hit = 0;
    }
    g_window[i] = (uint16_t)((g_window[i] << 1) | hit);
  }

  switch (g_state) {
  case ST_NORMAL: {
    uint8_t ramp_ready =
        det_debounced(0, RAMP_CONFIRM_M, RAMP_CONFIRM_N);
    if (ramp_ready) {
      enter_state(ST_RAMP, ti->det_value);
    }
    break;
  }

  case ST_RAMP: {
    if (g_frames_in_state >= RAMP_HOLD_FRAMES) {
      enter_state(ST_NORMAL, 0);
    }
    break;
  }

  case ST_RECOVERY: {
    if (ti->valid_rows >= RECOVERY_MIN_ROWS &&
        ti->both_lost_rows <= RECOVERY_MAX_BOTH_LOST) {
      if (g_exit_confirm < 255u) {
        g_exit_confirm++;
      }
      if (g_exit_confirm >= RECOVERY_CONFIRM_FRAMES) {
        enter_state(ST_NORMAL, (int16_t)ti->valid_rows);
      }
    } else {
      g_exit_confirm = 0;
    }

    if (g_state == ST_RECOVERY &&
        g_frames_in_state >= RECOVERY_TIMEOUT_FRAMES) {
      enter_state(ST_FAULT, -3);
    }
    break;
  }

  case ST_FAULT:
    break;

  default: {
    enter_state(ST_FAULT, -2);
    break;
  }
  }

  /* 元素超时兜底（当前仅坡道可超时进入恢复） */
  if (g_state != ST_NORMAL && g_state != ST_RECOVERY &&
      g_state != ST_FAULT && !g_just_entered) {
    uint32_t timeout = 0;
    if (g_state == ST_RAMP) {
      timeout = RAMP_TIMEOUT_FRAMES;
    }
    if (timeout != 0 && g_frames_in_state >= timeout) {
      enter_state(ST_RECOVERY, -1);
    }
  }

  return g_state;
}
