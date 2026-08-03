/* telemetry.c — 整圈数据记录 + 停车后 CSV 回传
 *
 * 设计取舍(为什么是"存下来再传"而不是实时流):
 *   记录期只做一次 16 字节的结构体赋值,几十个时钟,对 50 fps 的控制
 *   周期没有可测影响。实时流每帧要 snprintf + uart_write_buffer,既占
 *   控制周期,链路阻塞还会反过来拖慢控制 —— 这个工程以前踩过。
 *   车停下来之后再慢慢传,传多久都无所谓。
 *
 * 输出走的是 debug_init() 已经初始化好的调试串口(cpu0_main.c 开头就
 * 调过),所以不用配任何引脚。默认 115200,一圈 25 s 的数据约 55 KB,
 * 传完约 5 秒。如果你的 SDK 里 printf 没有接到串口,只需要把下面
 * log_puts() 一个函数换成 uart_write_buffer(UART_x, ...),其余不用动。
 *
 * 缓冲是线性的,不是环形:写满就停,并且把 overflow 一起打进表头。
 * 环形缓冲写满会覆盖开头,你拿到的就不是"整圈"而是"最后一段",而且
 * 看不出来 —— 默默丢数据比丢数据本身更糟。
 */
#include "telemetry.h"
#include "config.h"
#include "motor.h" /* hal_time_us */
#include "zf_common_headfile.h"
#include <stdio.h>

/* 一条 16 字节。字段顺序按对齐排,末尾显式补 1 字节,
   sizeof 不依赖编译器的填充策略。 */
typedef struct {
  uint32_t t_ms;      /*  0  相对开始记录的毫秒 */
  int16_t error;      /*  4  control_out_t.error_used */
  uint16_t servo;     /*  6  舵机 PWM 绝对值(SERVO_MIN..SERVO_MAX) */
  uint16_t duty;      /*  8  本帧实际下发的占空比(软启动段在爬) */
  uint8_t aim_row;    /* 10  瞄准行 tr;0 = 无有效行 */
  uint8_t lost_rows;  /* 11  双边丢线行数 */
  uint8_t err_hold;   /* 12  误差保持已持续帧数 */
  uint8_t threshold;  /* 13  本帧二值化阈值(Otsu 时逐帧在变) */
  uint8_t flags;      /* 14  bit0 drive_en, bit1 cross_valid */
  uint8_t reserved;   /* 15 */
} telem_rec_t;

/* config.h 里那张"RAM ↔ 可记录时长"的表是按 16 字节算的。改字段时
   这里先编译报错,而不是等上了赛道才发现只记了一半。 */
typedef char telem_rec_is_16_bytes[(sizeof(telem_rec_t) == 16) ? 1 : -1];

#define TELEM_FLAG_DRIVE_EN (0x01u)
#define TELEM_FLAG_CROSS (0x02u)

/* 落在默认数据段(= CPU0 DSPR),和 image.c 的静态量同一块。
   算上 image_bin 的 22.5 KB,120 KB 里用掉约 76 KB。
   若 ADS 链接报 cpu0 dsram 放不下,两条退路:
     1. 把 config.h 的 TELEM_MAX_FRAMES 改小(那张表写了对应时长)
     2. CPU1 现在完全空转,72 KB DSPR 一个字节没用 —— 给下面这行套上
        #pragma section all "cpu1_dsram" / #pragma section all restore,
        CPU0 经全局地址段写它。TC264 的 DSPR 跨核访问硬件强制非 cache,
        不需要额外同步。 */
static telem_rec_t s_log[TELEM_MAX_FRAMES];
static uint16_t s_count;
static uint8_t s_overflow;
static uint32_t s_t0_us;

/* 唯一的输出口。换串口只改这里。 */
static void log_puts(const char *s) { printf("%s", s); }

void telemetry_start(void) {
  s_count = 0;
  s_overflow = 0;
  s_t0_us = hal_time_us();
}

void telemetry_log(const track_info_t *ti, const control_out_t *out,
                   uint8_t drive_en) {
  telem_rec_t *r;

  if (s_count >= (uint16_t)TELEM_MAX_FRAMES) {
    s_overflow = 1;
    return;
  }

  r = &s_log[s_count];
  r->t_ms = (uint32_t)(hal_time_us() - s_t0_us) / 1000u;
  r->error = out->error_used;
  r->servo = out->servo_pwm;
  r->duty = out->duty;
  r->aim_row = ti->aim_row;
  r->lost_rows = ti->both_lost_rows;
  r->err_hold = ti->err_hold;
  r->threshold = ti->threshold;
  r->flags = (uint8_t)((drive_en ? TELEM_FLAG_DRIVE_EN : 0u) |
                       (ti->cross_valid ? TELEM_FLAG_CROSS : 0u));
  r->reserved = 0;
  s_count++;
}

uint16_t telemetry_count(void) { return s_count; }

uint8_t telemetry_overflow(void) { return s_overflow; }

void telemetry_dump(void) {
  char buf[96];
  uint16_t i;

  /* 表头把这一趟用的全部参数一起打出来 —— 每份 CSV 自带标定信息,
     事后不会出现"这条曲线是哪个 Kp 跑的"这种对不上号的事。
     浮点用 x1000 的整数打,不依赖 SDK 的 printf 支不支持 %f。 */
  snprintf(buf, sizeof(buf), "# CarRun log n=%u overflow=%u\r\n",
           (unsigned)s_count, (unsigned)s_overflow);
  log_puts(buf);

  snprintf(buf, sizeof(buf), "# kp_m=%d kd_m=%d alpha_m=%d (x1000)\r\n",
           (int)(steer_kp * 1000.0f), (int)(steer_kd * 1000.0f),
           (int)(steer_d_filt_alpha * 1000.0f));
  log_puts(buf);

  snprintf(buf, sizeof(buf),
           "# far=%u duty=%u th=%d cross=%u lostfr=%u stop=%u\r\n",
           (unsigned)steer_look_far, (unsigned)drive_duty_base,
           (int)image_threshold, (unsigned)image_cross_fill,
           (unsigned)drive_failsafe_frames, (unsigned)drive_stop_time_s);
  log_puts(buf);

  snprintf(buf, sizeof(buf), "# servo center=%d min=%d max=%d sat_err=%d\r\n",
           (int)SERVO_CENTER, (int)SERVO_MIN, (int)SERVO_MAX,
           (steer_kp > 0.01f) ? (int)((SERVO_MAX - SERVO_CENTER) / steer_kp)
                              : 0);
  log_puts(buf);

  /* en=0 的行不能当控制数据看:丢线锁死之后 cpu0_main 每帧都调
     control_init(),把 PD 的历史清零,所以那些行的 err/srv 不是一个
     正常运行的控制器会产生的值。留着它们是为了看触发瞬间的现场。 */
  log_puts("# rows with en=0 are post-failsafe: PD state is reset every\r\n");
  log_puts("# frame there, so err/srv are not valid controller output\r\n");
  log_puts("t_ms,err,srv,duty,aim,lost,hold,th,en,crs\r\n");

  for (i = 0; i < s_count; i++) {
    const telem_rec_t *r = &s_log[i];
    snprintf(buf, sizeof(buf), "%lu,%d,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
             (unsigned long)r->t_ms, (int)r->error, (unsigned)r->servo,
             (unsigned)r->duty, (unsigned)r->aim_row, (unsigned)r->lost_rows,
             (unsigned)r->err_hold, (unsigned)r->threshold,
             (unsigned)((r->flags & TELEM_FLAG_DRIVE_EN) ? 1u : 0u),
             (unsigned)((r->flags & TELEM_FLAG_CROSS) ? 1u : 0u));
    log_puts(buf);
  }

  log_puts("# end\r\n");
}
