# cpu1_main.c 逐行注释

> 行号与源文件一致。

CPU1 入口：关闭看门狗、使能全局中断、与 CPU0 同步就绪后进入空循环。当前工程未在 CPU1 上分配图像或控制任务，该核仅占位以满足双核启动流程。

---

```c
// L1: 文件标识注释。
// L2: （空行。）
// L3: 包含逐飞公共头文件（看门狗、中断、双核同步 API）。
// L4: `#pragma section all "cpu1_dsram"` — 将本文件代码/数据默认链到 CPU1 专用 DSRAM 段。
// L5: （空行。）
// L6: CPU1 入口函数 `core1_main`，由启动代码在核 1 上调用。
// L7: 开括号。
// L8: `disable_Watchdog()` — 关闭 CPU1 侧看门狗，避免空循环未喂狗复位（与 CPU0 策略一致）。
// L9: `interrupt_global_enable(0)` — 使能全局中断；参数 0 为逐飞库约定的 CPU 核编号。
// L10: （空行。）
// L11: `cpu_wait_event_ready()` — 阻塞直到双核同步事件就绪（与 CPU0 在 `core0_main` 中配对调用）。
// L12: `while (TRUE)` — 无限循环；CPU1 无其它业务，主动让出算力给 CPU0。
// L13: 开循环体。
// L14: （空行，循环体为空。）
// L15: 闭循环体 `}`。
// L16: 闭函数 `}`。
// L17: `#pragma section all restore` — 恢复默认 section，避免影响后续编译单元。
```

---

## 说明

- 智能车实时链路（摄像头、图像、PD、电机）全部在 **CPU0** 的 `cpu0_main.c` 中完成。
- CPU1 保留是为 TC264 双核工程模板兼容；若未来需要 offload（例如 IPS 刷新、SD 卡日志），可在此循环内添加任务，但须注意核间共享内存与 `hal_time_us` 的可见性。
