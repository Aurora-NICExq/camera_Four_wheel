/* cpu1_main.c — 人机界面:按键 + 菜单 + IPS200 屏幕
 *
 * 搬到这里的唯一理由:ips200 刷一帧 188x120 灰度图要推 45KB 过 SPI,
 * 以前这十几毫秒整条控制链都在干等。屏幕归本核独占,CPU0 一行显示代码都没有。
 * 本核不碰电机、不碰 control.c 的状态,只写调参量和请求标志(见 shared.h)。
 */

#include "config.h"
#include "image.h"
#include "menu.h"
#include "shared.h"
#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"

static void camera_view_draw(void) {
  const disp_frame_t *f = &shared_disp;

  image_debug_show(f->gray, &f->track);

  ips200_show_string(0, IMG_H + 4, "ERR");
  ips200_show_int(32, IMG_H + 4, f->error_used, 4);
  ips200_show_string(104, IMG_H + 4, "SRV");
  ips200_show_uint(136, IMG_H + 4, f->servo_pwm, 4);
  ips200_show_string(0, IMG_H + 20, "AIM");
  ips200_show_uint(32, IMG_H + 20, f->track.aim_row, 3);
  ips200_show_string(104, IMG_H + 20, "DTY");
  ips200_show_uint(136, IMG_H + 20, f->duty, 4);
  ips200_show_string(0, IMG_H + 36, "LST");
  ips200_show_uint(32, IMG_H + 36, f->track.both_lost_rows, 3);
  ips200_show_string(0, IMG_H + 52, "TH");
  ips200_show_uint(32, IMG_H + 52, f->track.threshold, 3);
  ips200_show_string(104, IMG_H + 52, "CRS");
  ips200_show_uint(136, IMG_H + 52, f->track.cross_valid, 1);
  ips200_show_string(0, IMG_H + 68, "HLD");
  ips200_show_uint(32, IMG_H + 68, f->track.err_hold, 3);
  ips200_show_string(104, IMG_H + 68, "FAR");
  ips200_show_uint(136, IMG_H + 68, f->look_far, 3);
  /* EN=0 就是 CPU0 锁死了驱动(丢线或本核心跳超时),不看这一格会以为车"没反应" */
  ips200_show_string(0, IMG_H + 84, "EN");
  ips200_show_uint(32, IMG_H + 84, f->drive_en, 1);
}

void core1_main(void) {
  disable_Watchdog();
  interrupt_global_enable(0);

  cpu_wait_event_ready();
  while (!shared_cpu0_ready) {
    /* 等 CPU0 把时钟和相机初始化完,再去碰 SPI 屏 */
  }

  menu_init(); /* 含 menu_port_init():ips200_init 在本核执行 */

  while (TRUE) {
    shared_cpu1_beat++; /* 心跳:CPU0 看不到它变就锁死驱动 */

    menu_task();

    if (menu_camera_view()) {
      if (shared_disp_ready) {
        camera_view_draw();
        shared_disp_ready = 0; /* 画完交还邮箱 */
      }
      shared_disp_req = 1; /* 要下一帧 */
    } else {
      shared_disp_req = 0;
    }
  }
}
#pragma section all restore
