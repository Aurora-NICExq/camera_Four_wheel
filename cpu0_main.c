/* cpu0_main.c — 实时链:摄像头 → 图像 → 控制 → 电机
 *
 * 按键/菜单/IPS200 屏幕全部在 CPU1(cpu1_main.c)。本核不许出现任何
 * ips200_* 调用:屏幕归 CPU1 独占,两个核同时驱动 SPI 会撞。
 * 共享数据和所有权规则见 shared.h。
 */
#include "config.h"
#include "control.h"
#include "image.h"
#include "menu.h"
#include "motor.h"
#include "shared.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

static track_info_t g_track;

int core0_main(void) {
  clock_init();
  debug_init();

  motor_hw_init();
  control_init();

  mt9v03x_init();
  shared_cpu0_ready = 1; /* 时钟和相机就绪,放行 CPU1 去初始化屏幕 */
  cpu_wait_event_ready();

  uint16_t fail_cnt = 0;
  uint32_t armed_elapsed_us = 0; //定时功能
  uint32_t armed_last_us = 0;
  uint8_t armed_t0_set = 0;
  uint8_t drive_en = 1;
  control_out_t out = {0};
  uint32_t cpu1_beat_prev = shared_cpu1_beat;
  uint32_t cpu1_beat_us = hal_time_us();

  while (TRUE) {
    if (!mt9v03x_finish_flag) {
      continue;
    }

    //图像处理

    image_process((const uint8_t (*)[IMG_W])mt9v03x_image, &g_track);

    {

    //失控保护

      if (g_track.aim_row == 0u) {
        if (fail_cnt < drive_failsafe_frames) {
          fail_cnt++;
        }
      } else {
        fail_cnt = 0;
      }
      /* 一旦触发就锁死:视野恢复也不再置回 drive_en,
         避免车"磕一下 → 自己又冲出去"。只有撤销 Armed 才解除(见循环末尾) */
      if (fail_cnt >= drive_failsafe_frames) {
        drive_en = 0;
      }
    }

    {

    //CPU1 失联保护

      /* 分核之后 CPU1 卡死不会再拖停主循环:屏幕定格、车照跑。
         心跳超时按丢线同样的方式锁死,解除条件也一样(撤 Armed)。
         注意 CPU1 画一帧灰度图要十几毫秒,阈值必须远大于它。 */
      uint32_t beat = shared_cpu1_beat;
      uint32_t now_us = hal_time_us();
      if (beat != cpu1_beat_prev) {
        cpu1_beat_prev = beat;
        cpu1_beat_us = now_us;
      } else if ((uint32_t)(now_us - cpu1_beat_us) > CPU1_ALIVE_TIMEOUT_US) {
        drive_en = 0;
      }
    }

    if (shared_ctrl_reset_req) {
      /* 菜单 Reset:控制器状态和电机归 CPU0 独占,CPU1 只能请求 */
      control_init();
      motor_reset();
      shared_ctrl_reset_req = 0;
    }

    control_update(&g_track, &out);

    //电机的处理

    if (menu_motor_test_mode()) {
      motor_apply(SERVO_CENTER, MOTOR_TEST_DUTY);
    } else if (menu_left_test_mode()) {
      motor_apply_left_only(MOTOR_TEST_DUTY);
    } else if (menu_align_test_mode()) {
      motor_apply_servo_only(out.servo_pwm);
    } else if (drive_en && drive_armed) {  //正常发车
      uint32_t now_us = hal_time_us();
      if (!armed_t0_set) {
        armed_t0_set = 1;
        armed_elapsed_us = 0;
      } else {
        uint32_t dt = now_us - armed_last_us;
        if (dt > DRIVE_DT_CLAMP_US) {
          dt = DRIVE_DT_NOMINAL_US;
        }
        armed_elapsed_us += dt;
      }
      armed_last_us = now_us;
      if (drive_timed_out ||
          armed_elapsed_us >=
              DRIVE_LAUNCH_DELAY_US + (uint32_t)drive_stop_time_s * 1000000u) {
        drive_timed_out = 1;
        motor_reset();
        control_duty_reset();
      } else if (armed_elapsed_us < DRIVE_LAUNCH_DELAY_US) {
        motor_reset();
        control_duty_reset();
      } else {
        motor_apply(out.servo_pwm, out.duty);
      }
    } else {
      motor_reset();
      if (!drive_en) {
        control_init();
      } else {
        control_duty_reset();
      }
    }

    if (!drive_armed) {
      armed_t0_set = 0;
      drive_timed_out = 0;
      /* 丢线锁死只在撤销 Armed 时解除,和发车超时同一个粒度。
         想改成"只有断电才解除",把下面两行删掉即可 */
      fail_cnt = 0;
      drive_en = 1;
    }

    /* CPU1 只在 Camera 页要图;不要就是一次拷贝都不做 */
    shared_serve_display(&g_track, &out, drive_en);

    mt9v03x_finish_flag = 0;
  }
}

#pragma section all restore
