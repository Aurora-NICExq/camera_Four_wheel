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
  uint8_t severe_fail_cnt = 0;
  uint32_t armed_t0_us = 0;
  uint8_t armed_t0_set = 0;
  uint8_t drive_en = 1;
  control_out_t out = {0};

  while (TRUE) {
    menu_task();

    if (!mt9v03x_finish_flag) {
      continue;
    }

    image_process((const uint8_t (*)[IMG_W])mt9v03x_image, control_duty_prev,
                  &g_track);

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

    if (menu_motor_test_mode()) {
      motor_apply(SERVO_CENTER, MOTOR_TEST_DUTY);
      control_duty_prev = 0;
    } else if (menu_align_test_mode()) {
      motor_apply_servo_only(out.servo_pwm);
      control_duty_prev = 0;
    } else if (drive_en && drive_armed) {
      uint32_t now_us = hal_time_us();
      if (!armed_t0_set) {
        armed_t0_set = 1;
        armed_t0_us = now_us;
      }
      if (drive_timed_out || (now_us - armed_t0_us) >= DRIVE_STOP_ELAPSED_US) {
        drive_timed_out = 1;
        motor_reset();
        control_duty_prev = 0;
        control_duty_reset();
      } else if ((now_us - armed_t0_us) < DRIVE_LAUNCH_DELAY_US) {
        motor_reset();
        control_duty_prev = 0;
        control_duty_reset();
      } else {
        motor_apply(out.servo_pwm, out.duty);
        control_duty_prev = out.duty;
      }
    } else {
      motor_reset();
      control_duty_prev = 0;
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

    if (menu_camera_view())
    {
      ips200_displayimage03x((const uint8 *)mt9v03x_image, IMG_W, IMG_H);
    }
    mt9v03x_finish_flag = 0;
  }
}

#pragma section all restore
