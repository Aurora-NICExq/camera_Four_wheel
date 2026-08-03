/* cpu0_main.c */
#include "config.h"
#include "control.h"
#include "image.h"
#include "menu.h"
#include "motor.h"
#include "menu_port.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

static track_info_t g_track;

int core0_main(void) {
  clock_init();
  debug_init();

  motor_hw_init();
  menu_port_init();
  control_init();
  menu_init();

  mt9v03x_init();
  cpu_wait_event_ready();

  uint16_t fail_cnt = 0;
  uint32_t armed_elapsed_us = 0; //定时功能
  uint32_t armed_last_us = 0;
  uint8_t armed_t0_set = 0;
  uint8_t drive_en = 1;
  control_out_t out = {0};

  while (TRUE) {
    menu_task();

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

    if (menu_camera_view()) {
      image_debug_show(&g_track);
      ips200_show_string(0, IMG_H + 4, "ERR");
      ips200_show_int(32, IMG_H + 4, out.error_used, 4);
      ips200_show_string(104, IMG_H + 4, "SRV");
      ips200_show_uint(136, IMG_H + 4, out.servo_pwm, 4);
      ips200_show_string(0, IMG_H + 20, "AIM");
      ips200_show_uint(32, IMG_H + 20, g_track.aim_row, 3);
      ips200_show_string(104, IMG_H + 20, "DTY");
      ips200_show_uint(136, IMG_H + 20, out.duty, 4);
      ips200_show_string(0, IMG_H + 36, "LST");
      ips200_show_uint(32, IMG_H + 36, g_track.both_lost_rows, 3);
      ips200_show_string(0, IMG_H + 52, "TH");
      ips200_show_uint(32, IMG_H + 52, g_track.threshold, 3);
      ips200_show_string(104, IMG_H + 52, "CRS");
      ips200_show_uint(136, IMG_H + 52, g_track.cross_valid, 1);
      ips200_show_string(0, IMG_H + 68, "HLD");
      ips200_show_uint(32, IMG_H + 68, g_track.err_hold, 3);
      ips200_show_string(104, IMG_H + 68, "FAR");
      ips200_show_uint(136, IMG_H + 68, steer_look_far, 3);
    }
    mt9v03x_finish_flag = 0;
  }
}

#pragma section all restore
