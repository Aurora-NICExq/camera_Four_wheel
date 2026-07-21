/* cpu0_main.c */
#include "config.h"
#include "control.h"
#include "image.h"
#include "menu.h"
#include "motor.h"
#include "menu_port.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

static uint16_t g_duty_prev;
static track_info_t g_track;

int core0_main(void) {
  clock_init();
  debug_init();

  motor_hw_init();
  menu_port_init();
  control_init();
  menu_init();
  g_duty_prev = 0;

  mt9v03x_init();
  cpu_wait_event_ready();

  uint16_t fail_cnt = 0;
  uint8_t severe_fail_cnt = 0;
  uint8_t drive_en = 1;
  control_out_t out = {0};

  while (TRUE) {
    if (!mt9v03x_finish_flag) {
      continue;
    }

    image_process((const uint8_t (*)[IMG_W])mt9v03x_image, g_duty_prev,
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

    if (drive_en) {
      motor_apply(out.servo_pwm, out.duty);
      g_duty_prev = out.duty;
    } else {
      motor_stop();
      g_duty_prev = 0;
      control_init();
    }

    menu_task();
    mt9v03x_finish_flag = 0;
  }
}

#pragma section all restore
