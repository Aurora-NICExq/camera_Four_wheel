/*********************************************************************************************************************
 * 文件：cpu0_main.c — CPU0 主流程：初始化 → 解锁 → 帧同步主循环 → 失控保护
 * （基于逐飞 TC264 开源库 cpu0_main.c 模板改写；库版权声明见 libraries/LICENSE，GPL3.0）
 *
 * 主循环严格帧同步：等 mt9v03x_finish_flag → 处理 → 清标志 → 等下一帧。
 * 帧就是全系统唯一时钟（FRAMES_PER_SECOND），任何逻辑不使用毫秒。
 *
 * 职责边界：本文件只做"搬运与门闸"——
 *   图像结果 → fsm → control → motor 的数据搬运；
 *   解锁/失控/调试断油三道门闸；
 *   绝不出现任何元素判断逻辑（那些只属于 fsm.c 的唯一 switch）。
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "config.h"
#include "image.h"
#include "fsm.h"
#include "control.h"
#include "motor.h"
#include "display.h"
#include "perf.h"

#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

/* 帧间共享（仅本文件内）：上一帧下发占空比 —— image_process 的权重混合输入 */
static uint16_t     g_duty_prev;
static track_info_t g_track;        /* 每帧图像结果（约 750 字节，静态分配，无动态内存） */

/*-------------------------------------------------------------------------------------------------------------------
 * startup_beep — 上电 2 s 等待 + 蜂鸣提示（发生在相机启动前，是全工程唯一的毫秒计时）
 * 为什么等：给舵机/驱动上电稳定时间，也给人把车放上赛道的时间。
 *------------------------------------------------------------------------------------------------------------------*/
static void startup_beep(void)
{
    uint32 i;
    for (i = 0; i < (STARTUP_DELAY_MS / (STARTUP_BEEP_MS * 5u)); i++)
    {
        hal_buzzer_on();
        system_delay_ms(STARTUP_BEEP_MS);
        hal_buzzer_off();
        system_delay_ms(STARTUP_BEEP_MS * 4u);
    }
}

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口

    /* ---- 外设初始化（顺序：执行器先归零位，再开显示，最后开相机） ---- */
    motor_hw_init();                // 舵机中位 + 电机 0 + 蜂鸣器 + 按键 + 系统定时器
    display_init();                 // IPS200 + 无线串口
    fsm_init();
    control_init();
    g_duty_prev = 0;

    startup_beep();                 // 2 s 上电等待

    if (mt9v03x_init() != 0)
    {
        /* 相机初始化失败：长鸣并停在这里 —— 没有图像的车不允许进入主循环 */
        while (TRUE)
        {
            hal_buzzer_on();
            system_delay_ms(100);
            hal_buzzer_off();
            system_delay_ms(100);
        }
    }

    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    uint8_t  armed      = 0;        /* 解锁状态：0 = 电机永远 0                            */
    uint16_t fail_cnt   = 0;        /* 失控保护连续帧计数（任何机制不得屏蔽它）             */
    uint8_t  severe_fail_cnt = 0;   /* 全白/严重过曝快速保护计数                           */
    uint32_t last_frame_us = hal_time_us(); /* 相机活性看门狗；独立于帧时基                  */
    control_out_t out   = {0};
#if TEST_COAST
    uint8_t coast_cut   = 0;        /* 滑行标定：1 = 已触发切油                             */
#endif

    while (TRUE)
    {
        /* ================= 帧同步：没有新帧就等（唯一的"空转"点） ================= */
        if (!mt9v03x_finish_flag)
        {
            /* 帧时基在相机失联时不会前进，因此这里必须使用独立 STM 时间。
             * 无符号减法天然兼容 32 位计时器回绕。 */
            if (armed && (uint32_t)(hal_time_us() - last_frame_us) >= CAMERA_WATCHDOG_US)
            {
                armed = 0;
                motor_stop();
                display_chirp(ST_FAULT);  /* 相机恢复出帧后播放长鸣；武装标志保持 SAFE */
            }
            continue;
        }

        last_frame_us = hal_time_us();

        uint32_t t0 = hal_time_us();
        PERF_BEGIN(PF_TOTAL);

        /* ================= 1. 图像流水线（纯逻辑，与 replay 逐位一致） ================= */
        image_process((const uint8_t (*)[IMG_W])mt9v03x_image, g_duty_prev, &g_track);

        /* ================= 2. 状态机（唯一 switch 在 fsm.c 内） ================= */
        PERF_BEGIN(PF_FSM);
        fsm_state_t st = fsm_update(&g_track);
        PERF_END(PF_FSM);
        if (fsm_state_just_entered())
        {
            display_chirp(st);      /* fsm 无 I/O：提示音由这里代发 */
        }

        /* ================= 3. 按键：解锁 / 翻页 ================= */
        hal_key_scan();
        if (hal_key_pressed(KEY_IDX_ARM))
        {
            armed = (uint8_t)!armed;
            if (armed)
            {
                /* 重新解锁 = 全新一轮：状态机、控制器、失控计数一起复位；
                 * 占空比从 0 沿升斜坡爬升（软启动就是斜坡本身） */
                fsm_init();
                control_init();
                fail_cnt = 0;
                severe_fail_cnt = 0;
#if TEST_COAST
                coast_cut = 0;
#endif
            }
            else
            {
                motor_stop();
            }
        }
        if (hal_key_pressed(KEY_IDX_PAGE))
        {
            display_next_page();
        }

        /* ================= 4. 失控保护（最高优先级，永不被去抖/冷却/状态屏蔽） ================= */
        uint8_t severe_image = 0;
        if (image_track_invalid(&g_track, &severe_image))
        {
            if (fail_cnt < FAILSAFE_FRAMES)
            {
                fail_cnt++;
            }
        }
        else
        {
            fail_cnt = 0;
        }
        if (severe_image)
        {
            if (severe_fail_cnt < FAILSAFE_SEVERE_FRAMES)
            {
                severe_fail_cnt++;
            }
        }
        else
        {
            severe_fail_cnt = 0;
        }
        if (armed && (fail_cnt >= FAILSAFE_FRAMES ||
                      severe_fail_cnt >= FAILSAFE_SEVERE_FRAMES))
        {
            armed = 0;              /* 全黑/全白/冲出赛道：断油上锁，等待人工重新解锁 */
            motor_stop();
            display_chirp(ST_FAULT);/* 长鸣提示图像失效；保持上锁直到人工重新解锁 */
        }

        /* FSM 元素阶段超时后会先低速恢复；恢复仍失败才请求解除武装。 */
        if (armed && fsm_fault_request())
        {
            armed = 0;
            motor_stop();
            display_chirp(ST_FAULT);
        }

        /* ================= 5. 控制律 ================= */
        PERF_BEGIN(PF_CONTROL);
        control_update(&g_track, armed, &out);
        PERF_END(PF_CONTROL);

#if TEST_COAST
        /* 滑行标定模式：恒速巡航，按键或黑标记条触发切油；转向照常工作。
         * 触发后 out.duty 直接覆写 —— 标定要的是阶跃切油，不是斜坡。 */
        if (armed)
        {
            if (hal_key_pressed(KEY_IDX_COAST) || g_track.valid_rows < TEST_COAST_MARKER_ROWS)
            {
                coast_cut = 1;
            }
            out.duty = coast_cut ? TEST_COAST_TARGET_DUTY : TEST_COAST_CRUISE_DUTY;
        }
#endif

        /* ================= 6. 输出门闸 → 硬件 ================= */
        {
            uint16_t duty_final = out.duty;
#if DEBUG_NO_DRIVE
            duty_final = 0;         /* 调试模式：完整流水线 + 丰富显示，唯独不给油 */
#endif
            motor_apply(out.servo_pwm, duty_final);
            g_duty_prev = duty_final;
        }

        /* ================= 7. 耗时测量 + 显示/遥测（分频限流，不阻塞） ================= */
        PERF_END(PF_TOTAL);
        uint32_t proc_us = hal_time_us() - t0;
        PERF_BEGIN(PF_DISPLAY);
        display_update((const uint8_t (*)[IMG_W])mt9v03x_image, &g_track, &out, proc_us, armed);
        display_telemetry(&g_track, &out, fsm_frame_count());
        PERF_END(PF_DISPLAY);
        PERF_COMMIT();          /* 本帧各阶段耗时结算；报表帧顺带发送统计（见 perf.c） */

        /* ================= 8. 释放本帧，等待下一帧 ================= */
        mt9v03x_finish_flag = 0;
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
