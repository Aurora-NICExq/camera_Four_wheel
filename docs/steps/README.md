# 智能车摄像头循迹固件：手把手学习教程

> 本教程对应当前分支上的**完整可烧录业务代码**（根目录 `.c/.h` 与 `output_user_code/` 内容一致）。
> 赛道目标：**直道 + 转弯**；数据流 **图像 → 控制 → 电机**，无状态机、无编码器速度环。
>
> 阅读方式：按 step0 → step6 顺序学习；每步末尾有「自查清单」。上车前务必完成 step6。

---

## 1. 这套代码是什么

| 项目 | 说明 |
|------|------|
| **平台** | Infineon AURIX TC264D + SeekFree 开源库 |
| **感知** | MT9V03X 灰度摄像头 188×120 @ 50fps |
| **执行** | 舵机转向 + 双路电机同速前进（开环占空比） |
| **人机** | IPS200 竖屏 240×320 调试菜单 + 无线串口遥测 |
| **安全** | 上电默认上锁、`DEBUG_NO_DRIVE`、图像失效断油、相机看门狗 |

本仓库**不含** ADS 工程壳（`.project`、逐飞库、链接脚本）。烧录时把源码拷入 ADS 工程的 `user/` 目录，见 [step6](./step6_ADS烧录与上车验收.md)。

---

## 2. 单向数据流（必须记住）

```text
mt9v03x_image (DMA 灰度帧)
        │
        ▼
image_process()  ──► track_info_t（边线/中线/误差/健康）
        │
        ▼
control_update() ──► control_out_t（servo_pwm / duty）
        │
        ▼
cpu0_main 门闸（armed / DEBUG_NO_DRIVE / 失控保护）
        │
        ▼
motor_apply()    ──► PWM + GPIO

并行支路（不反馈进控制）：
  menu_task()     ──► IPS200 调参
  display_*()     ──► 遥测 + 蜂鸣
```

四条边界（违反即架构腐化）：

1. `image.c` / `control.c` **不包含**逐飞或 MCU 头文件；
2. **只有** `motor.c` 直接写 PWM/GPIO；
3. **只有** `cpu0_main.c` 持有 `armed` 并做最终断油门闸；
4. 菜单只写 `volatile` 全局，控制/图像每帧读同一份。

---

## 3. 文件地图

```text
test_CarRun/
├── config.h              全部可调宏（唯一参数源）
├── pins.h                引脚与 IPS200 接口类型
├── image.c/h             阈值 + 边线跟踪 + 加权误差
├── hybrid_8n_longest_col/  八邻域双边跟踪实现
├── control.c/h           转向 PD + 定速开环占空比
├── motor.c/h             舵机/电机/按键/蜂鸣器 HAL
├── display.c/h           IPS200 初始化 + 遥测 + 蜂鸣
├── menu*.c/h             数据驱动调参菜单
├── cpu0_main.c           帧同步主循环与安全门闸
├── isr*.c/h              逐飞模板中断（相机 DMA 等）
├── cpu1_main.c           CPU1 空循环
└── output_user_code/     与根目录相同的 user 交付包（拷 ADS 用）
```

---

## 4. 学习路线

| 步骤 | 文档 | 你会搞懂什么 |
|------|------|--------------|
| 0 | [工程总览与目录](./step0_工程总览与目录.md) | 分层、时钟、坐标约定 |
| 1 | [配置与引脚](./step1_配置与引脚.md) | `config.h` / `pins.h` 每一项的含义 |
| 2 | [图像流水线](./step2_图像流水线.md) | 大津法、hybrid 跟踪、加权误差 |
| 3 | [控制律](./step3_控制律.md) | PD、Kp 调度、斜坡、armed |
| 4 | [电机、显示与菜单](./step4_电机显示与菜单.md) | HAL、竖屏菜单、Flash 存参 |
| 5 | [主循环与安全门闸](./step5_主循环与安全门闸.md) | 帧同步、失控保护、解锁 |
| 6 | [ADS 烧录与上车验收](./step6_ADS烧录与上车验收.md) | 导入 ADS、调试顺序、验收表 |

---

## 5. 与 `test_version_codex/docs/steps` 的关系

旧迁移教程包含 **FSM、环岛、perf** 等已删除模块。本套教程只描述**当前精简版**：

- 无 `fsm.c`、无 `perf.c`、无速度 PID；
- 主循环不再经过状态机；
- IPS200 为**竖屏 240×320**（菜单 30×20 字符）。

若旧文档提到 `ST_ZEBRA`、`fsm_update()`，以本仓库源码为准。

---

## 6. 建议学习方式

1. **先读 step0～1**，对照 `config.h` 把宏抄一遍注释；
2. **读 step2 时**打开 `image.c` + `hybrid_track.c`，用菜单调 `Threshold` 观察边线变化；
3. **读 step3～5 时**保持 `DEBUG_NO_DRIVE=1`，只解锁舵机观察转向；
4. **最后 step6** 在 ADS 编译通过后再改 `DEBUG_NO_DRIVE=0` 低速试跑。

每完成一步，用自查清单打勾后再进入下一步。
