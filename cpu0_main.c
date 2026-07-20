/* cpu0_main.c - CPU0 init, unlock, frame-sync loop, failsafe */
#include "config.h"
#include "control.h"
#include "display.h"
#include "fsm.h"
#include "image.h"
#include "menu.h"
#include "motor.h"
#include "zf_common_headfile.h"

/* 数据流：图像 → 状态机契约 → 控制 → 电机；失控保护独立于元素状态。 */

#pragma section all "cpu0_dsram"

static uint16_t g_duty_prev;
static track_info_t g_track;

static volatile uint8_t g_arm_toggle_req;
void menu_action_arm(void) { g_arm_toggle_req = 1u; }

static void startup_beep(void) {
  uint32 i;
  for (i = 0; i < (STARTUP_DELAY_MS / (STARTUP_BEEP_MS * 5u)); i++) {
    hal_buzzer_on();
    system_delay_ms(STARTUP_BEEP_MS);
    hal_buzzer_off();
    system_delay_ms(STARTUP_BEEP_MS * 4u);
  }
}

int core0_main(void) {
  clock_init(); // 获取时钟频率<务必保留>
  debug_init(); // 初始化默认调试串口

  /* ---- 外设初始化（顺序：执行器先归零位，再开显示，最后开相机） ---- */
  motor_hw_init();
  display_init();
  control_init();
  fsm_init();
  menu_init();
  g_duty_prev = 0;

  startup_beep(); // 2 s 上电等待

  if (mt9v03x_init() != 0) {
    /* 相机初始化失败：长鸣并停在这里 —— 没有图像的车不允许进入主循环 */
    while (TRUE) {
      hal_buzzer_on();
      system_delay_ms(100);
      hal_buzzer_off();
      system_delay_ms(100);
    }
  }

  cpu_wait_event_ready(); // 等待所有核心初始化完毕

  uint8_t armed = 0; /* 解锁状态：0 = 电机永远 0                            */
  uint16_t fail_cnt = 0; /* 失控保护连续帧计数（任何机制不得屏蔽它） */
  uint8_t severe_fail_cnt = 0; /* 全白/严重过曝快速保护计数 */
  uint32_t frame_count = 0; /* 本文件自己的帧计数（遥测行号 + 分频时基） */
  uint32_t last_frame_us = hal_time_us(); /* 相机活性看门狗；独立于帧时基 */
  control_out_t out = {0};
#if TEST_COAST
  uint8_t coast_cut = 0; /* 滑行标定：1 = 已触发切油 */
#endif

  while (TRUE) {
    /* 帧同步：无新帧则等（唯一空转点） */
    if (!mt9v03x_finish_flag) {
      /* 帧时基在相机失联时不会前进，因此这里必须使用独立 STM 时间。
       * 无符号减法天然兼容 32 位计时器回绕。 */
      if (armed &&
          (uint32_t)(hal_time_us() - last_frame_us) >= CAMERA_WATCHDOG_US) {
        armed = 0;
        motor_stop();
        display_chirp_fault(); /* 相机恢复出帧后播放长鸣；武装标志保持 SAFE */
      }
      continue;
    }

    last_frame_us = hal_time_us();
    frame_count++;

    image_process((const uint8_t (*)[IMG_W])mt9v03x_image, g_duty_prev,
                  &g_track);

    {
      fsm_state_t st = fsm_update(&g_track);
      if (fsm_state_just_entered()) {
        display_chirp(st);
      }
    }

    if (g_arm_toggle_req) {
      g_arm_toggle_req = 0;
      armed = (uint8_t)!armed;
      if (armed) {
        /* 重新解锁 = 全新一轮：控制器（PD 记忆 + 斜坡）、失控计数一起复位；
         * 占空比从 0 沿升斜坡爬升（软启动就是斜坡本身） */
        control_init();
        fsm_init();
        fail_cnt = 0;
        severe_fail_cnt = 0;
#if TEST_COAST
        coast_cut = 0;
#endif
      } else {
        motor_stop();
      }
    }

    /* 失控保护（最高优先级） */
    uint8_t severe_image = 0;
    if (image_track_invalid(&g_track, &severe_image)) {
      if (fail_cnt < FAILSAFE_FRAMES) {
        fail_cnt++;
      }
    } else {
      fail_cnt = 0;
    }
    if (severe_image) {
      if (severe_fail_cnt < FAILSAFE_SEVERE_FRAMES) {
        severe_fail_cnt++;
      }
    } else {
      severe_fail_cnt = 0;
    }
    if (armed && (fail_cnt >= FAILSAFE_FRAMES ||
                  severe_fail_cnt >= FAILSAFE_SEVERE_FRAMES)) {
      armed = 0; /* 全黑/全白/冲出赛道：断油上锁，等待人工重新解锁 */
      motor_stop();
      display_chirp_fault(); /* 长鸣提示图像失效；保持上锁直到人工重新解锁 */
    }

    if (armed && fsm_fault_request()) {
      armed = 0;
      motor_stop();
      display_chirp_fault();
    }

    control_update(&g_track, armed, &out);

#if TEST_COAST
    /* 滑行标定模式：恒速巡航，按键或黑标记条触发切油；转向照常工作。
     * 触发后 out.duty 直接覆写 —— 标定要的是阶跃切油，不是斜坡。 */
    if (armed) {
      if (hal_key_pressed(KEY_IDX_COAST) ||
          g_track.valid_rows < TEST_COAST_MARKER_ROWS) {
        coast_cut = 1;
      }
      out.duty = coast_cut ? TEST_COAST_TARGET_DUTY : TEST_COAST_CRUISE_DUTY;
    }
#endif

    {
      uint16_t duty_final = out.duty;
#if DEBUG_NO_DRIVE
      duty_final = 0; /* 调试模式：完整流水线 + 丰富显示，唯独不给油 */
#endif
      motor_apply(out.servo_pwm, duty_final);
      g_duty_prev = duty_final;
    }

    /* 菜单 + 遥测 + 蜂鸣（分频，不阻塞） */
    menu_task();      /* 扫键 + 脏行重绘 */
    display_update(); /* 蜂鸣器提示音（不碰屏幕） */
    display_telemetry(&g_track, &out, frame_count);

    mt9v03x_finish_flag = 0;
  }
}

#pragma section all restore
