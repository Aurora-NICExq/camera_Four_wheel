#include "config.h"
#include "control.h"
#include "image.h"
#include "menu.h"
#include "motor.h"
#include "menu_port.h"
#include "telemetry.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

static track_info_t g_track;

int core0_main(void) {
  clock_init();
  debug_init();

  motor_hw_init();
  menu_port_init();
  control_init();
  telemetry_init();
  menu_init();

  mt9v03x_init();
  cpu_wait_event_ready();

  uint16_t fail_cnt = 0;
  uint8_t severe_fail_cnt = 0;
  uint32_t armed_elapsed_us = 0;
  uint32_t armed_last_us = 0;
  uint8_t armed_t0_set = 0;
  uint8_t drive_en = 1;
  uint32_t telem_frame = 0;
  control_out_t out = {0};

  uint32_t ut_seq = 0, ut_drop = 0, ut_last_us = 0;
  uint8_t ut_active_prev = 0, ut_tick = 0;

  while (TRUE) {
    menu_task();


    ut_tick = 0;
    if (menu_uart_test_mode()) {
      uint32_t now_us = hal_time_us();
      if (!ut_active_prev) {
        ut_active_prev = 1;
        ut_seq = 0;
        ut_drop = 0;
        ut_last_us = now_us;
        ut_tick = 1;
        if (!telemetry_test_banner()) {
          ut_drop++;
        }
      } else if ((uint32_t)(now_us - ut_last_us) >= UART_TEST_PERIOD_US) {
        ut_last_us = now_us;
        ut_seq++;
        ut_tick = 1;
        if (!telemetry_test_send(ut_seq, now_us / 1000u)) {
          ut_drop++;
        }
      }
    } else {
      ut_active_prev = 0;
    }
    telemetry_pump();

    if (menu_uart_test_mode() && ut_tick) {
      menu_port_draw_uint(8, 2, telemetry_wireless_ok(), 7, MENU_STYLE_NORMAL);
      menu_port_draw_uint(8, 3, telemetry_tx_bytes(), 7, MENU_STYLE_NORMAL);
      menu_port_draw_uint(8, 4, ut_seq, 7, MENU_STYLE_NORMAL);
      menu_port_draw_uint(8, 5, ut_drop, 7, MENU_STYLE_NORMAL);
      menu_port_draw_uint(8, 6, telemetry_queue_depth(), 7, MENU_STYLE_NORMAL);
      menu_port_draw_uint(8, 7, telemetry_rts_blocked(), 7, MENU_STYLE_NORMAL);

      menu_port_draw_uint(8, 8, telemetry_baud(), 7, MENU_STYLE_NORMAL);
    }

    if (!mt9v03x_finish_flag) {
      continue;
    }

    image_process((const uint8_t (*)[IMG_W])mt9v03x_image, &g_track);

    {
      uint8_t severe_image = 0;
      if (image_track_invalid(&g_track, &severe_image)) {
        if (fail_cnt < FAILSAFE_FRAMES) {
          fail_cnt++;
        }
      } else {
        fail_cnt = 0;
        drive_en = 1;
      }
      if (severe_image) {
        if (severe_fail_cnt < FAILSAFE_SEVERE_FRAMES) {
          severe_fail_cnt++;
        }
      } else {
        severe_fail_cnt = 0;
      }
      if (fail_cnt >= FAILSAFE_FRAMES ||
          severe_fail_cnt >= FAILSAFE_SEVERE_FRAMES) {
        drive_en = 0;
      }
    }

    control_update(&g_track, &out);

    telem_frame++;

    if (drive_armed && !menu_uart_test_mode())
    {
      uint32_t telem_t_ms = armed_elapsed_us / 1000u;
      telemetry_update(telem_t_ms, telem_frame, &g_track, &out);
    }

    if (menu_uart_test_mode()) {
      motor_reset();
    } else if (menu_motor_test_mode()) {
      motor_apply(SERVO_CENTER, MOTOR_TEST_DUTY);
    } else if (menu_align_test_mode()) {
      motor_apply_servo_only(out.servo_pwm);
    } else if (drive_en && drive_armed) {
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
    }


    if (menu_calib_view()) {
      uint8_t th = image_calib_show((const uint8_t (*)[IMG_W])mt9v03x_image);
      ips200_show_string(0, IMG_H + 4, "TH");
      if (image_threshold > 0) {
        ips200_show_int(32, IMG_H + 4, image_threshold, 3);
      } else {
        ips200_show_string(32, IMG_H + 4, "A");
        ips200_show_int(48, IMG_H + 4, th, 3);
      }
      ips200_show_string(0, IMG_H + 20, "NO 3X3 FLT");
      ips200_show_string(0, IMG_H + 36, "UP/DN:TH");
      ips200_show_string(0, IMG_H + 52, "BACK:exit");
    } else if (menu_camera_view()) {
      image_debug_show(&g_track);
      ips200_show_string(0, IMG_H + 4, "ERR");
      ips200_show_int(32, IMG_H + 4, out.error_used, 4);
      ips200_show_string(104, IMG_H + 4, "SRV");
      ips200_show_uint(136, IMG_H + 4, out.servo_pwm, 4);
      ips200_show_string(0, IMG_H + 20, "ROW");
      ips200_show_uint(32, IMG_H + 20, g_track.valid_rows, 3);
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
      ips200_show_string(88, IMG_H + 68, "N/F/A");
      ips200_show_uint(136, IMG_H + 68, g_track.near_rows, 2);
      ips200_show_uint(152, IMG_H + 68, g_track.far_rows, 2);
      ips200_show_uint(168, IMG_H + 68, g_track.aim_rows, 2);
    }
    mt9v03x_finish_flag = 0;
  }
}

#pragma section all restore
