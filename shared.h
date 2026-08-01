/* shared.h — CPU0 / CPU1 共享数据
 *
 * 分核:
 *   CPU0 (TriCore 1.6P,超标量,120KB DSPR) = 摄像头 + 图像 + 控制 + 电机
 *   CPU1 (TriCore 1.6E,标量小核,72KB DSPR) = 按键 + 菜单 + IPS200 屏幕
 * 图像必须留在 CPU0:相机 DMA 缓冲和全部中断(isr_config.h 里都是
 * IfxSrc_Tos_cpu0)都在 CPU0 侧,且 1.6P 比 1.6E 快。搬走的是屏幕——
 * ips200 刷一帧 188x120 灰度图要走 45KB SPI,以前整条控制链都在等它。
 *
 * 为什么这些变量不加 #pragma section:
 *   TC264 的 LMU 是 0 KB(datasheet),两个核的 DSPR 之外没有第三块内存,
 *   "单独开辟一块共享内存区"没有物理载体。共享块落在默认段(CPU0 DSPR),
 *   CPU1 经全局地址段 7H 访问。TriCore 架构手册 8.3.3/8.3.4:段 DH(本地
 *   DSPR)与段 0H-7H(别核 DSPR 镜像)硬件强制非 cache,只有 8H/9H 可 cache。
 *   所以跨核共享没有 cache 一致性问题,volatile 就够,不用动 .lsl。
 *   烧录前在 .map 里确认共享符号地址是 0x7xxxxxxx(全局),不是 0xDxxxxxxx
 *   (本地)——落在本地地址上会变成"两个核各读各的副本"。
 *
 * 所有权规则(违反就会看到撕裂的图或抢不到的外设):
 *   1. 调参量(steer_kp / drive_armed / image_threshold / ...):CPU1 写,CPU0 读。
 *      都是 ≤32 位对齐标量,单次读写原子,不需要锁。
 *   2. 电机 PWM 和 control.c 的内部状态:CPU0 独占。CPU1 要复位只能置
 *      shared_ctrl_reset_req,由 CPU0 执行。
 *   3. 调试帧 shared_disp:单槽邮箱,归属由 req/ready 转移(见下),
 *      任何时刻只有一个核碰它,所以不需要双缓冲。
 *   4. ips200:CPU1 独占。CPU0 一行显示代码都不许有。
 */
#ifndef SHARED_H
#define SHARED_H

#include <stdint.h>
#include "config.h"
#include "control.h"
#include "image.h"

/* CPU0 → CPU1 的调试帧。灰度图和边线取自同一帧,所以画出来不会错位。 */
typedef struct {
  uint8_t gray[IMG_H][IMG_W];
  track_info_t track;
  int16_t error_used;
  uint16_t servo_pwm;
  uint16_t duty;
  uint16_t look_far;
  uint8_t drive_en;
} disp_frame_t;

extern disp_frame_t shared_disp;

/* 单槽邮箱协议:
 *   ready==0 && req==1 → 归 CPU0:填 shared_disp,然后 req=0、ready=1
 *   ready==1           → 归 CPU1:画完置 ready=0,再置 req=1 要下一帧
 * CPU1 不在 Camera 页时不置 req,CPU0 就一次也不拷贝(省掉每帧 22.5KB memcpy)。 */
extern volatile uint8_t shared_disp_req;
extern volatile uint8_t shared_disp_ready;

/* CPU0 外设初始化(时钟/相机)完成。CPU1 必须等到 1 才能碰 SPI 屏。 */
extern volatile uint8_t shared_cpu0_ready;

/* CPU1 心跳:每轮主循环自增。CPU0 超过 CPU1_ALIVE_TIMEOUT_US 没看到变化
 * 就按丢线同样的方式锁死驱动——分核之后 CPU1 卡死不再自动停车,必须补这个。 */
extern volatile uint32_t shared_cpu1_beat;

/* CPU1 → CPU0 的一次性请求:复位控制器和电机(菜单 Reset)。CPU0 执行后清零。 */
extern volatile uint8_t shared_ctrl_reset_req;

/* CPU0 侧调用:邮箱空且有请求时填一帧。没请求就直接返回。 */
void shared_serve_display(const track_info_t *ti, const control_out_t *out,
                          uint8_t drive_en);

#endif /* SHARED_H */
