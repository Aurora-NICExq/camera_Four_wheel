/* fsm.c - sole element FSM (pure logic) */
#include "fsm.h"
#include "config.h"
#include "image.h"
#include <stdint.h>

#define FSM_STATIC_ASSERT(cond, tag) typedef char fsm_static_assert_##tag[(cond) ? 1 : -1]

/* contract table: left-ring defaults; [] length checked vs ST_COUNT */
static const state_contract_t g_contract_base[] = {
    /* ST_NORMAL    */ {STEER_SRC_MIDLINE, 0, DUTY_HARD_CAP, 0},
    /* ST_CROSS     */
    {STEER_SRC_MIDLINE, 0, CROSS_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
    /* ST_RING_PRE  */
    {STEER_SRC_RIGHT_EDGE, 0, RING_PRE_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
    /* ST_RING_IN   */
    {STEER_SRC_RIGHT_EDGE, 0, RING_IN_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
    /* ST_RING_EXIT */
    {STEER_SRC_FIXED_BIAS, +RING_EXIT_BIAS_PX, RING_EXIT_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
    /* ST_RAMP      */
    {STEER_SRC_MIDLINE, 0, RAMP_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
    /* ST_RECOVERY  */
    {STEER_SRC_MIDLINE, 0, RECOVERY_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
    /* ST_FAULT     */
    {STEER_SRC_MIDLINE, 0, 0,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
};

/* 右环镜像表（仅环岛三态与基础表不同：巡线边与偏置符号取镜像）。
 * 用第二张 const 表而不是运行时改写 ——
 * 契约表保持只读，任何路径都无法意外修改它。 */
static const state_contract_t g_contract_ring_right[3] = {
    /* ST_RING_PRE  */ {STEER_SRC_LEFT_EDGE, 0, RING_PRE_DUTY_CAP,
                        DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT |
                            DET_BIT_RAMP},
    /* ST_RING_IN   */
    {STEER_SRC_LEFT_EDGE, 0, RING_IN_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
    /* ST_RING_EXIT */
    {STEER_SRC_FIXED_BIAS, -RING_EXIT_BIAS_PX, RING_EXIT_DUTY_CAP,
     DET_BIT_CROSS | DET_BIT_RING_LEFT | DET_BIT_RING_RIGHT | DET_BIT_RAMP},
};

/* fsm.h 的承诺在此兑现：契约表必须为每个状态各留一行。
 * 新增 ST_* 却漏填契约行时，g_contract_base 的长度不再等于 ST_COUNT，
 * 本断言令编译立即失败 —— 而不是等到运行期被 fsm_update 的 default 分支悄悄兜底、
 * 或让新状态拿到一行全 0 契约（duty_cap=0，直接趴窝）而无人察觉。 */
FSM_STATIC_ASSERT(sizeof(g_contract_base) / sizeof(g_contract_base[0]) == ST_COUNT,
                  contract_base_covers_all_states);
FSM_STATIC_ASSERT(CROSS_CONFIRM_N >= 1 && CROSS_CONFIRM_N <= 16 &&
                      CROSS_CONFIRM_M >= 1 && CROSS_CONFIRM_M <= CROSS_CONFIRM_N,
                  cross_debounce_config_valid);
FSM_STATIC_ASSERT(RING_CONFIRM_N >= 1 && RING_CONFIRM_N <= 16 &&
                      RING_CONFIRM_M >= 1 && RING_CONFIRM_M <= RING_CONFIRM_N,
                  ring_debounce_config_valid);
FSM_STATIC_ASSERT(RAMP_CONFIRM_N >= 1 && RAMP_CONFIRM_N <= 16 &&
                      RAMP_CONFIRM_M >= 1 && RAMP_CONFIRM_M <= RAMP_CONFIRM_N,
                  ramp_debounce_config_valid);

/* 二、内部状态 */

static fsm_state_t g_state;        /* 当前状态        */
static uint32_t g_frame;           /* 累计帧号（唯一时基）           */
static uint32_t g_frames_in_state; /* 当前状态已持续帧数（超时判定） */
static uint8_t g_just_entered; /* 本帧刚切换状态（control.c 的 D 项防踢读取） */
static uint16_t
    g_cooldown; /* 冷却剩余帧数（>0 时 COOLDOWN_MASK 内检测器禁止进入）  */

/* 去抖窗口：每个检测器一个 16 位移位寄存器，bit0 = 最新帧是否命中 */
#define DET_NUM (4)
static uint16_t g_window[DET_NUM];

/* 环岛方向与半宽快照 */
static uint8_t g_ring_is_left; /* 1 = 左环 */
static uint8_t
    g_ring_width[IMG_H]; /* 进入 RING_PRE 时的逐行宽度表快照：环内另一侧是伪边，
                            实时宽度不可信，必须用"进入前"的透视关系 */

/* 迟滞-退出的连续确认计数器（进入阈值与退出阈值分离 = 结构性迟滞） */
static uint8_t g_exit_confirm;
static uint8_t g_phase_confirm; /* 环岛阶段内部转移确认计数器 */

/* 转移轨迹环形缓冲 */
static fsm_trace_entry_t g_trace[FSM_TRACE_LEN];
static uint8_t g_trace_head;  /* 下一条写入位置  */
static uint8_t g_trace_count; /* 已写入条数（≤ FSM_TRACE_LEN） */

static uint8_t g_fault_req;

/* 三、内部工具 */

static uint8_t popcount16(uint16_t v) {
  uint8_t n = 0;
  while (v) {
    n = (uint8_t)(n + (v & 1u));
    v >>= 1;
  }
  return n;
}

/* 单帧误检在结构上无法引发转移 */
static uint8_t det_debounced(uint8_t det_idx, uint8_t confirm_m,
                             uint8_t confirm_n) {
  uint16_t mask = (confirm_n == 16u)
                      ? 0xFFFFu
                      : (uint16_t)((1u << confirm_n) - 1u);
  return (uint8_t)(popcount16((uint16_t)(g_window[det_idx] & mask)) >=
                   confirm_m);
}

/* fsm */
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
  g_phase_confirm = 0;
  if (to == ST_FAULT) {
    g_fault_req = 1;
  }
  g_just_entered = 1;
}

static uint8_t count_lost_rows(const uint8_t *lost, uint8_t lo, uint8_t hi,
                               uint8_t valid) {
  uint8_t r;
  uint8_t n = 0;
  if (hi > valid) {
    hi = valid;
  }
  for (r = lo; r < hi; r++) {
    if (lost[r]) {
      n++;
    }
  }
  return n;
}

/* 四、对外接口 */

void fsm_init(void) {
  uint8_t i;
  g_state = ST_NORMAL;
  g_frame = 0;
  g_frames_in_state = 0;
  g_just_entered = 0;
  g_cooldown = 0;
  g_ring_is_left = 1;
  g_exit_confirm = 0;
  g_phase_confirm = 0;
  g_trace_head = 0;
  g_trace_count = 0;
  g_fault_req = 0;
  for (i = 0; i < DET_NUM; i++) {
    g_window[i] = 0;
  }
  for (i = 0; i < IMG_H; i++) {
    g_ring_width[i] = WIDTH_TABLE_DEFAULT;
  }
}

fsm_state_t fsm_state(void) { return g_state; }

uint8_t fsm_state_just_entered(void) { return g_just_entered; }

uint8_t fsm_ring_is_left(void) { return g_ring_is_left; }

uint8_t fsm_fault_request(void) { return g_fault_req; }

uint32_t fsm_frame_count(void) { return g_frame; }

uint16_t fsm_ring_half_width(uint8_t row) {
  if (row >= IMG_H) {
    row = IMG_H - 1u;
  }
  return (uint16_t)(g_ring_width[row] / 2u);
}

const state_contract_t *fsm_contract(void) {
  if ((uint8_t)g_state >= (uint8_t)ST_COUNT) {
    return &g_contract_base[ST_FAULT];
  }
  /* 右环时环岛三态取镜像表；其余状态（含左环）一律用基础表 */
  if (!g_ring_is_left) {
    if (g_state == ST_RING_PRE) {
      return &g_contract_ring_right[0];
    }
    if (g_state == ST_RING_IN) {
      return &g_contract_ring_right[1];
    }
    if (g_state == ST_RING_EXIT) {
      return &g_contract_ring_right[2];
    }
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

  /* --- 去抖窗口更新 --- */
  const state_contract_t *ct = fsm_contract();
  uint8_t raw[DET_NUM];
  raw[0] = ti->det_cross;
  raw[1] = ti->det_ring_left;
  raw[2] = ti->det_ring_right;
  raw[3] = ti->det_ramp;

  uint8_t mask_bits = ct->detector_mask;
  if (g_cooldown > 0) {
    mask_bits |= COOLDOWN_MASK;
  }

  uint8_t i;
  for (i = 0; i < DET_NUM; i++) {
    uint8_t hit = raw[i];
    if (mask_bits & (uint8_t)(1u << i)) {
      hit = 0; /* 被屏蔽的检测器连窗口都不积累，解除屏蔽后需重新凑满 M/N */
    }
    g_window[i] = (uint16_t)((g_window[i] << 1) | hit);
  }

  /* --- 唯一 switch：全部转移都在这里 --- */
  switch (g_state) {
  case ST_NORMAL: {
    uint8_t ring_left_ready =
        det_debounced(1, RING_CONFIRM_M, RING_CONFIRM_N);
    uint8_t ring_right_ready =
        det_debounced(2, RING_CONFIRM_M, RING_CONFIRM_N);
    uint8_t cross_ready =
        det_debounced(0, CROSS_CONFIRM_M, CROSS_CONFIRM_N);
    uint8_t ramp_ready =
        det_debounced(3, RAMP_CONFIRM_M, RAMP_CONFIRM_N);

    /* 进入仲裁：固定优先级 环岛 > 十字 > 坡道。
     * 左右环同时完成去抖属于方向冲突，不能默认猜左环；此时放弃环岛候选，
     * 让更明确的十字判定继续参与仲裁。
     */
    if ((uint8_t)(ring_left_ready ^ ring_right_ready)) {
      uint8_t last_good_width = WIDTH_TABLE_DEFAULT;
      g_ring_is_left = ring_left_ready;
      /* 半宽快照：进入环岛序列的这一刻，宽度表还是"直道透视"的可信值。
       * 仅复制有效且合法的行；无效远行沿用最后一个可信宽度，禁止把未初始化数组带入环内。 */
      for (i = 0; i < IMG_H; i++) {
        if (i < ti->valid_rows && ti->width[i] >= WIDTH_MIN_PX &&
            ti->width[i] <= WIDTH_MAX_PX) {
          last_good_width = ti->width[i];
        }
        g_ring_width[i] = last_good_width;
      }
      enter_state(ST_RING_PRE, ti->det_value);
    } else if (cross_ready) {
      enter_state(ST_CROSS, ti->det_value);
    } else if (ramp_ready) {
      enter_state(ST_RAMP, ti->det_value);
    }
    break;
  }

  case ST_CROSS: {
    /* 退出（迟滞）：进入靠"双边丢失 ≥ CROSS_MIN_BOTH_LOST"，
     * 退出靠"双边丢失 ≤ CROSS_EXIT_MAX_LOST 且连续 CROSS_EXIT_CONFIRM 帧" ——
     * 两套阈值。 */
    if (ti->both_lost_rows <= CROSS_EXIT_MAX_LOST) {
      g_exit_confirm++;
      if (g_exit_confirm >= CROSS_EXIT_CONFIRM) {
        enter_state(ST_NORMAL, (int16_t)ti->both_lost_rows);
      }
    } else {
      g_exit_confirm = 0;
    }
    break;
  }

  case ST_RING_PRE: {
    /* 入环时机：弧侧（环口侧）缺口压到车头近处 —— 近行带内弧侧丢线过半。 */
    const uint8_t *arc_lost = g_ring_is_left ? ti->left_lost : ti->right_lost;
    uint8_t near_lost =
        count_lost_rows(arc_lost, 0, RING_ENTRY_ROW, ti->valid_rows);
    if (near_lost >= (RING_ENTRY_ROW / 2u)) {
      if (g_phase_confirm < 255u) {
        g_phase_confirm++;
      }
      if (g_phase_confirm >= RING_ENTRY_CONFIRM) {
        enter_state(ST_RING_IN, (int16_t)near_lost);
      }
    } else {
      g_phase_confirm = 0;
    }
    break;
  }

  case ST_RING_IN: {
    /* 出环口出现：环内沿【外侧】边线巡线（左环=右边线）；绕回环口时外侧边线出现大段断口。
     * RING_IN_MIN_FRAMES
     * 内屏蔽此判定：入环口自己的断口还没离开视野，会被误认成出环口。 */
    if (g_frames_in_state >= RING_IN_MIN_FRAMES) {
      const uint8_t *solid_lost =
          g_ring_is_left ? ti->right_lost : ti->left_lost;
      uint8_t break_rows = count_lost_rows(solid_lost, RING_BAND_ROW_LO,
                                           RING_BAND_ROW_HI, ti->valid_rows);
      if (break_rows >= RING_EXIT_BREAK_LOST) {
        if (g_phase_confirm < 255u) {
          g_phase_confirm++;
        }
        if (g_phase_confirm >= RING_EXIT_BREAK_CONFIRM) {
          enter_state(ST_RING_EXIT, (int16_t)break_rows);
        }
      } else {
        g_phase_confirm = 0;
      }
    } else {
      g_phase_confirm = 0;
    }
    break;
  }

  case ST_RING_EXIT: {
    /* 出环完成（迟滞）：双边恢复且丢失 ≤ RING_EXIT_DONE_MAX_LOST，连续
     * RING_EXIT_CONFIRM 帧。 */
    if (ti->both_lost_rows <= RING_EXIT_DONE_MAX_LOST &&
        ti->valid_rows > CURV_NEAR_ROW_HI) {
      g_exit_confirm++;
      if (g_exit_confirm >= RING_EXIT_CONFIRM) {
        enter_state(ST_NORMAL, (int16_t)ti->both_lost_rows);
      }
    } else {
      g_exit_confirm = 0;
    }
    break;
  }

  case ST_RAMP: {
    /* 坡道：限速保持固定帧数后恢复（占位逻辑，赛季规则定型后完善退出特征）。 */
    if (g_frames_in_state >= RAMP_HOLD_FRAMES) {
      enter_state(ST_NORMAL, 0);
    }
    break;
  }

  case ST_RECOVERY: {
    /* 元素超时只表示阶段证据丢失，不等价于成功通过。
     * 低速沿当前中线寻找；必须连续看到足够前瞻且双边恢复后才回 NORMAL。 */
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
      enter_state(ST_FAULT, -3); /* 恢复失败：吸收态 + 请求解除武装 */
    }
    break;
  }

  case ST_FAULT:
    /* 吸收态；main 读取 fsm_fault_request() 后解除武装，重新解锁时 fsm_init() 清除。 */
    break;

  default: {
    /* 内存破坏等不可达情况不得恢复供油：进入吸收故障态。 */
    enter_state(ST_FAULT, -2);
    break;
  }
  }

  /* --- 超时兜底：对仍处于元素处理状态的状态统一执行（恢复态、故障态除外） --- */
  if (g_state != ST_NORMAL && g_state != ST_RECOVERY &&
      g_state != ST_FAULT && !g_just_entered) {
    uint32_t timeout = 0;
    switch (g_state) {
    case ST_CROSS:
      timeout = CROSS_TIMEOUT_FRAMES;
      break;
    case ST_RING_PRE:
      timeout = RING_PRE_TIMEOUT_FRAMES;
      break;
    case ST_RING_IN:
      timeout = RING_IN_TIMEOUT_FRAMES;
      break;
    case ST_RING_EXIT:
      timeout = RING_EXIT_TIMEOUT_FRAMES;
      break;
    case ST_RAMP:
      timeout = RAMP_TIMEOUT_FRAMES;
      break;
    default:
      timeout = 0;
      break;
    }
    if (timeout != 0 && g_frames_in_state >= timeout) {
      enter_state(ST_RECOVERY,
                  -1); /* trigger=-1 标记"元素超时进入恢复"，轨迹页一眼可见 */
    }
  }

  return g_state;
}
