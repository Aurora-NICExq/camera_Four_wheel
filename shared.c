/* shared.c — 双核共享数据的唯一定义处。
 * 不加 #pragma section:落在默认段(CPU0 DSPR),CPU1 走全局地址访问,
 * 硬件保证非 cache。理由见 shared.h 顶部。 */
#include "shared.h"

/* ADS 用 Tasking 编译器,不认 GCC 的 __asm volatile("" ::: "memory")。
 * __memory_changed() 是 Tasking 内置函数;GCC 走 asm 屏障。 */
#if defined(__TASKING__)
#pragma intrinsic(__memory_changed)
#define SHARED_MEM_BARRIER() __memory_changed()
#elif defined(__GNUC__)
#define SHARED_MEM_BARRIER() __asm volatile("" ::: "memory")
#else
#define SHARED_MEM_BARRIER() ((void)0)
#endif

disp_frame_t shared_disp;

volatile uint8_t shared_disp_req = 0;
volatile uint8_t shared_disp_ready = 0;
volatile uint8_t shared_cpu0_ready = 0;
volatile uint32_t shared_cpu1_beat = 0;
volatile uint8_t shared_ctrl_reset_req = 0;

void shared_serve_display(const track_info_t *ti, const control_out_t *out,
                          uint8_t drive_en) {
  if (!shared_disp_req || shared_disp_ready) {
    return; /* 没人要,或者上一帧 CPU1 还没画完 */
  }

  image_copy_bin(shared_disp.gray);
  shared_disp.track = *ti;
  shared_disp.error_used = out->error_used;
  shared_disp.servo_pwm = out->servo_pwm;
  shared_disp.duty = out->duty;
  shared_disp.look_far = steer_look_far;
  shared_disp.drive_en = drive_en;

  /* 编译器屏障:上面填结构体的都是普通写,标准只保证 volatile 之间不重排,
     不保证普通写不会被挪到 ready=1 之后。挪过去 CPU1 就会画到半帧。
     硬件侧不用管:非 cache 写,同一个从设备,SRI 保序。 */
  SHARED_MEM_BARRIER();

  shared_disp_req = 0;
  shared_disp_ready = 1;
}
