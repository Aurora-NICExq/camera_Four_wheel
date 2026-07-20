# 第 5 步：主循环与安全门闸——cpu0_main 是唯一调度者

> **目标**：读懂 `core0_main()` 每一段的职责，理解帧同步与三道安全门闸。
> 对应文件：`cpu0_main.c`、`cpu1_main.c`

---

## 1. main 只做「搬运与门闸」

```text
                    MT9V03X 摄像头
                          │
                          ▼
┌─────────────────────────────────────────────┐
│              cpu0_main.c                    │
│  等新帧 → image → control → motor           │
│         → menu / display                    │
└───────┬──────────┬──────────┬───────────────┘
        ▼          ▼          ▼
     image.c   control.c   motor.c
```

main **不**重复实现：阈值、PD、PWM 写入、菜单绘制。

---

## 2. 头文件 = 依赖清单

```c
#include "zf_common_headfile.h"  /* 时钟、相机、多核 */
#include "config.h"
#include "image.h"
#include "control.h"
#include "motor.h"
#include "display.h"
#include "menu.h"
```

无 `fsm.h`、无 `perf.h`——精简版已删除。

---

## 3. 跨帧变量

```c
static uint16_t     g_duty_prev;   /* 上一帧门闸后 duty → image 权重 */
static track_info_t g_track;       /* ~750B，静态分配 */
static volatile uint8_t g_arm_toggle_req;  /* 菜单 Arm 动作 */
```

`#pragma section all "cpu0_dsram"` 把 CPU0 热数据放本核 RAM。

---

## 4. 初始化顺序

```text
clock_init / debug_init
motor_hw_init      ← 执行器先归零
display_init       ← IPS200 竖屏
control_init
menu_init          ← 占满屏幕
startup_beep       ← 2s，唯一上电等待
mt9v03x_init       ← 失败则死循环闪鸣
cpu_wait_event_ready
```

---

## 5. 主循环七段（每帧）

### ① 帧同步

```c
if (!mt9v03x_finish_flag) {
    /* 相机看门狗：armed 且 100ms 无帧 → 断油 */
    continue;
}
```

没有新帧就空转等待——**唯一的 busy-wait**。

### ② 图像

```c
image_process(mt9v03x_image, g_duty_prev, &g_track);
```

### ③ 解锁

菜单 `Arm/Disarm` → `menu_action_arm()` → `g_arm_toggle_req`  
main 消费后切换 `armed`，解锁时 `control_init()` + 清零失控计数。

### ④ 失控保护（最高优先级）

```c
image_track_invalid(&g_track, &severe_image)
→ 累计 fail_cnt / severe_fail_cnt
→ 超限：armed=0, motor_stop(), display_chirp_fault()
```

### ⑤ 控制

```c
control_update(&g_track, armed, &out);
```

### ⑥ 输出门闸

```c
duty_final = out.duty;
#if DEBUG_NO_DRIVE
duty_final = 0;
#endif
motor_apply(out.servo_pwm, duty_final);
g_duty_prev = duty_final;
```

### ⑦ 菜单 + 遥测 + 清标志

```c
menu_task();
display_update();
display_telemetry(...);
mt9v03x_finish_flag = 0;
```

---

## 6. 三道门闸对照

| 门闸 | 触发 | 效果 |
|------|------|------|
| `armed==0` | 上电默认 / 菜单 Disarm / 失控 | control 内 duty=0；main 也可 motor_stop |
| `DEBUG_NO_DRIVE` | 编译期宏 | main 强制 duty_final=0 |
| 失控保护 | 图像失效 / 看门狗 | armed=0 + stop + 长鸣 |

三者**独立**：例如 `DEBUG_NO_DRIVE=1` 时仍可 Arm，但电机始终无油。

---

## 7. CPU1

`cpu1_main.c`：关看门狗、开中断、`cpu_wait_event_ready()` 后与 CPU0 同步，然后空循环。**无业务逻辑**。

---

## 8. TEST_COAST（默认关闭）

`config.h` 中 `TEST_COAST=0`。若置 1：恒速巡航 + 按键/黑标记切油，用于滑行标定，覆盖 `out.duty`。

---

## 9. 自查

- [ ] 能按顺序说出主循环 7 段
- [ ] 知道 `g_duty_prev` 是门闸后的值
- [ ] 知道按键由 `menu_task` 扫描，main 不直接扫键（除 TEST_COAST）
- [ ] 知道相机 init 失败不会进入主循环

下一步：[ADS 烧录与上车验收](./step6_ADS烧录与上车验收.md)
