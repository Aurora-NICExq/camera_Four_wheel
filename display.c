/*********************************************************************************************************************
 * 模块：display.c — 预算受限的调试输出（IPS200 屏 + 逐飞无线串口 + 蜂鸣器提示音）
 *
 * 硬规则：任何屏幕/串口写入都不允许阻塞帧同步主循环。实现手段：
 *   - 帧计数分频：文本页 2 Hz（DISPLAY_TEXT_DIV），图像页每 DISPLAY_IMG_DIV 帧一次；
 *   - 比赛模式（DEBUG_NO_DRIVE=0）绝不画图，只刷新一小块文本；
 *   - 串口整帧图像仅调试模式编译（ENABLE_UART_IMAGE），且每帧只发半行（94 字节
 *     ≈ 8 ms @115200）——整帧要 4.8 s，只用于静态检查二值化效果，绝不在跑车时开。
 *
 * 使用的逐飞 API（已核对真实头文件签名）：
 *   ips200_init / ips200_clear / ips200_show_string / ips200_show_int / ips200_show_uint /
 *   ips200_show_gray_image / ips200_draw_point           （zf_device_ips200.h）
 *   wireless_uart_init / wireless_uart_send_buffer        （zf_device_wireless_uart.h）
 ********************************************************************************************************************/
#include <stdio.h>
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "image.h"
#include "fsm.h"
#include "control.h"
#include "motor.h"
#include "display.h"

/*===================================================================================================================
 * 内部状态
 *==================================================================================================================*/

static uint32_t g_frame;            /* 本模块自己的帧计数（分频时基）                     */
static uint8_t  g_page;             /* 当前页：0 总览 / 1 图像叠加（调试模式）/ 2 FSM 轨迹 */
static uint8_t  g_chirp_frames;     /* 蜂鸣器剩余鸣叫帧数                                 */

/* 状态名表（与 fsm_state_t 顺序一致；屏幕与遥测共用） */
static const char *const g_state_name[ST_COUNT] =
{
    "NORMAL", "RAMP  ", "RECOV ", "FAULT ",
};

/*===================================================================================================================
 * 对外接口
 *==================================================================================================================*/

void display_init(void)
{
    ips200_init(IPS200_CONNECT_TYPE);
    ips200_clear();
    wireless_uart_init();
    g_frame        = 0;
    g_page         = 0;
    g_chirp_frames = 0;
}

void display_next_page(void)
{
    g_page = (uint8_t)((g_page + 1u) % 3u);
    ips200_clear();                     /* 换页清屏一次可以接受：由按键触发，非每帧路径 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * display_chirp — 状态进入提示音：中=坡道/恢复，长=故障。
 *------------------------------------------------------------------------------------------------------------------*/
void display_chirp(fsm_state_t state)
{
    switch (state)
    {
    case ST_RAMP:       g_chirp_frames = CHIRP_FRAMES_MID;   break;
    case ST_RECOVERY:   g_chirp_frames = CHIRP_FRAMES_MID;   break;
    case ST_FAULT:      g_chirp_frames = CHIRP_FRAMES_LONG;  break;
    default:                                                 break;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * page_overview — 第 0 页：一屏关键量（2 Hz 刷新，比赛模式唯一的屏幕开销）
 *------------------------------------------------------------------------------------------------------------------*/
static void page_overview(const track_info_t *ti, const control_out_t *out,
                          uint32_t proc_us, uint8_t armed)
{
    ips200_show_string(0,   0, armed ? "ARMED " : "SAFE  ");
    ips200_show_string(0,  20, "STATE:");
    ips200_show_string(96, 20, g_state_name[fsm_state()]);
    ips200_show_string(0,  40, "ERR  :");
    ips200_show_int   (96, 40, out->error_used, 4);
    ips200_show_string(0,  60, "DUTY :");
    ips200_show_uint  (96, 60, out->duty, 5);
    ips200_show_string(0,  80, "TARG :");
    ips200_show_uint  (96, 80, out->duty_target, 5);
    ips200_show_string(0, 100, "ROWS :");
    ips200_show_uint  (96, 100, ti->valid_rows, 3);
    ips200_show_string(0, 120, "CURV :");
    ips200_show_int   (96, 120, ti->curvature, 4);
    ips200_show_string(0, 140, "TH   :");
    ips200_show_uint  (96, 140, ti->threshold, 3);
    ips200_show_string(0, 160, "US   :");
    ips200_show_uint  (96, 160, proc_us, 5);       /* 流水线耗时：逼近帧周期就该调 COL_STEP 了 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * page_image — 第 1 页（仅调试模式）：二值化图像 + 中线/边线叠加，降采样一半
 * ips200_show_gray_image 的 threshold 参数非 0 时即按阈值二值显示 —— 直接复用逐飞实现。
 *------------------------------------------------------------------------------------------------------------------*/
#if DEBUG_NO_DRIVE
static void page_image(const uint8_t img[IMG_H][IMG_W], const track_info_t *ti)
{
    ips200_show_gray_image(0, 0, &img[0][0], IMG_W, IMG_H,
                           DISPLAY_IMG_W, DISPLAY_IMG_H, ti->threshold);

    /* 叠加边线/中线：每 4 行画一个点（性能预算）。逻辑行 near=0 在图像底部，
     * 显示 y 从上往下，故 y = (IMG_H-1-row) 再按比例缩放。 */
    uint8_t r;
    for (r = 0; r < ti->valid_rows; r += 4)
    {
        uint16_t y = (uint16_t)((uint32_t)(IMG_H - 1u - r) * DISPLAY_IMG_H / IMG_H);
        uint16_t xl = (uint16_t)((uint32_t)ti->left[r]  * DISPLAY_IMG_W / IMG_W);
        uint16_t xm = (uint16_t)((uint32_t)ti->mid[r]   * DISPLAY_IMG_W / IMG_W);
        uint16_t xr = (uint16_t)((uint32_t)ti->right[r] * DISPLAY_IMG_W / IMG_W);
        ips200_draw_point(xl, y, RGB565_BLUE);
        ips200_draw_point(xm, y, RGB565_RED);
        ips200_draw_point(xr, y, RGB565_GREEN);
    }
}
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * page_trace — 第 2 页：FSM 转移轨迹（最近 8 条：from→to @帧号 触发值；-1=超时强退）
 *------------------------------------------------------------------------------------------------------------------*/
static void page_trace(void)
{
    uint8_t i;
    ips200_show_string(0, 0, "FSM TRACE (new->old)");
    for (i = 0; i < 8u; i++)
    {
        const fsm_trace_entry_t *e = fsm_trace(i);
        uint16_t y = (uint16_t)(20u + i * 20u);
        if (e == (const fsm_trace_entry_t *)0)
        {
            ips200_show_string(0, y, "----------------    ");
            continue;
        }
        ips200_show_string(0,   y, g_state_name[e->from]);
        ips200_show_string(56,  y, ">");
        ips200_show_string(64,  y, g_state_name[e->to]);
        ips200_show_uint  (120, y, e->frame, 7);
        ips200_show_int   (184, y, e->trigger, 4);
    }
}

#if (ENABLE_UART_IMAGE && DEBUG_NO_DRIVE)
/*-------------------------------------------------------------------------------------------------------------------
 * uart_image_pump — 每帧发送半行图像（分块非阻塞策略）：
 * 帧头 0x00,0xFF,0x01,0x01（逐飞上位机协议风格）后逐块推进；发完整幅自动从头开始。
 *------------------------------------------------------------------------------------------------------------------*/
static void uart_image_pump(const uint8_t img[IMG_H][IMG_W])
{
    static uint16_t chunk = 0;                          /* 半行序号：0 .. IMG_H*2-1 */
    static const uint8_t header[4] = { 0x00, 0xFF, 0x01, 0x01 };

    if (chunk == 0)
    {
        wireless_uart_send_buffer(header, 4);
    }
    uint16_t row = chunk / 2u;
    uint16_t col = (uint16_t)((chunk % 2u) * (IMG_W / 2u));
    wireless_uart_send_buffer(&img[row][col], IMG_W / 2u);
    chunk = (uint16_t)((chunk + 1u) % (IMG_H * 2u));
}
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * display_telemetry — 遥测一行文本。比赛模式按 TELEMETRY_DIV 限流；
 * TEST_COAST 模式每帧输出 (frame, valid_rows, duty) —— 滑行标定的原始数据。
 *------------------------------------------------------------------------------------------------------------------*/
void display_telemetry(const track_info_t *ti, const control_out_t *out, uint32_t frame)
{
#if ENABLE_UART_TELEMETRY
    char buf[96];
    int len;
#if TEST_COAST
    /* 标定格式固定三列，便于上位机/脚本直接解析 */
    len = snprintf(buf, sizeof(buf), "%lu,%u,%u\r\n",
                   (unsigned long)frame, (unsigned)ti->valid_rows, (unsigned)out->duty);
    if (len > 0)
    {
        wireless_uart_send_buffer((const uint8 *)buf, (uint32)len);
    }
#else
    if ((frame % TELEMETRY_DIV) == 0u)
    {
        len = snprintf(buf, sizeof(buf), "F%lu S%s E%d R%u C%d D%u\r\n",
                       (unsigned long)frame, g_state_name[fsm_state()],
                       (int)out->error_used, (unsigned)ti->valid_rows,
                       (int)ti->curvature, (unsigned)out->duty);
        if (len > 0)
        {
            wireless_uart_send_buffer((const uint8 *)buf, (uint32)len);
        }
    }
#endif
#else
    (void)ti; (void)out; (void)frame;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * display_update — 每帧调用；内部分频，绝大多数帧只处理蜂鸣器后立即返回
 *------------------------------------------------------------------------------------------------------------------*/
void display_update(const uint8_t img[IMG_H][IMG_W], const track_info_t *ti,
                    const control_out_t *out, uint32_t proc_us, uint8_t armed)
{
    g_frame++;

    /* 蜂鸣器提示音：帧计数控制时长，永不阻塞 */
    if (g_chirp_frames > 0)
    {
        g_chirp_frames--;
        hal_buzzer_on();
    }
    else
    {
        hal_buzzer_off();
    }

#if (ENABLE_UART_IMAGE && DEBUG_NO_DRIVE)
    uart_image_pump(img);
#endif

    switch (g_page)
    {
    case 0:
        if ((g_frame % DISPLAY_TEXT_DIV) == 0u)
        {
            page_overview(ti, out, proc_us, armed);
        }
        break;

    case 1:
#if DEBUG_NO_DRIVE
        if ((g_frame % DISPLAY_IMG_DIV) == 0u)
        {
            page_image(img, ti);
        }
#else
        /* 比赛模式没有图像页：任何画图都会威胁帧预算，直接降级为总览 */
        if ((g_frame % DISPLAY_TEXT_DIV) == 0u)
        {
            page_overview(ti, out, proc_us, armed);
        }
#endif
        break;

    default:
        if ((g_frame % DISPLAY_TEXT_DIV) == 0u)
        {
            page_trace();
        }
        break;
    }

#if !DEBUG_NO_DRIVE
    (void)img;      /* 比赛模式不读原始图像 */
#endif
}
