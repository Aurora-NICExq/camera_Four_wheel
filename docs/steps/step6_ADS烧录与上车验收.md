# 第 6 步：ADS 烧录与上车验收

> **目标**：把本仓库源码导入 ADS 工程、编译烧录，并按顺序完成静态与动态验收。
> 参考硬件工程：`~/Developer/MotorServo_TC264`（逐飞 TC264 模板）

---

## 1. 你需要准备什么

| 组件 | 说明 |
|------|------|
| ADS v1.10.x | Infineon AURIX Development Studio |
| SeekFree TC264 库 | 随 ADS 工程提供（`libraries/`） |
| 本仓库源码 | 根目录或 `output_user_code/`（二者一致） |

本仓库**不包含** `.project`、链接脚本、逐飞库——需基于已有 ADS 工程导入。

---

## 2. 导入步骤

### 2.1 复制 user 源码

将以下内容拷入 ADS 工程的 `user/`（覆盖旧 user 业务文件，**保留**逐飞模板的 `Cpu0_Main.h` 若工程需要）：

```text
config.h  control.c/h  cpu0_main.c  cpu0_main.h  cpu1_main.c
display.c/h  image.c/h  isr.c/h  isr_config.h
menu.c/h  menu_config.c  menu_port.c/h  motor.c/h  pins.h
hybrid_8n_longest_col/hybrid_track.c
hybrid_8n_longest_col/hybrid_track.h
```

或直接：

```bash
cp -R output_user_code/*  <你的ADS工程>/user/
cp -R output_user_code/hybrid_8n_longest_col  <你的ADS工程>/user/
```

### 2.2 ADS 工程设置

1. 在 ADS 中把上述 `.c` 加入 CPU0 编译（hybrid_track.c 不要忘记）
2. Include 路径包含 `user/` 与 `user/hybrid_8n_longest_col`
3. 确认 `zf_common_headfile.h`、MT9V03X、IPS200 驱动已在工程中
4. **不要**改 `#include` 路径——本代码已按逐飞习惯写相对路径

### 2.3 引脚核对

对照 `pins.h` 与实车接线（可参考 `MotorServo_TC264/user/pins.h` 电机/舵机部分）。  
IPS200 使用 **SPI 竖屏**；`display_init` 已设 `IPS200_PORTAIT`。

---

## 3. 编译前检查清单

- [ ] `DEBUG_NO_DRIVE` 为 `1`（`config.h`）
- [ ] `IMG_W/H` 与 `MT9V03X_W/H` 一致（否则 `motor.c` `#error`）
- [ ] 舵机/电机 PWM 分属 ATOM1 / ATOM0
- [ ] 无残留旧文件（`fsm.c`、`pid.c`、`perf.c` 勿加入工程）

---

## 4. 上电验收顺序

### 阶段 A：静态（车架空转或架空）

| 步骤 | 操作 | 预期 |
|------|------|------|
| A1 | 上电 | 2s 蜂鸣，屏幕出现菜单 |
| A2 | 菜单 UP/DOWN | 光标移动，数值区不越界、不 assert 卡死 |
| A3 | 保持 Disarm | 电机无输出 |
| A4 | Arm | 仍 `DEBUG_NO_DRIVE=1` 时电机仍无油；舵机随转向变化 |
| A5 | 无线串口 | 约 2Hz 收到 `F... E... R... D...` |

### 阶段 B：图像与参数

| 步骤 | 操作 | 预期 |
|------|------|------|
| B1 | Threshold=0 | 大津法，误差随赛道变化 |
| B2 | 手动 Threshold | 阴影场景可稳定边线 |
| B3 | 调 Kp/Kd | 舵机响应变化，无瞬间打满 |

### 阶段 C：低速试跑

1. `config.h` 设 `DEBUG_NO_DRIVE` 为 `0`，重新编译
2. `STRAIGHT_DUTY` 从低值起（如 3000）逐步升高
3. 菜单 Arm → 观察直道/弯道
4. 遮摄像头 → 应在约 10 帧内断油 + 长鸣

### 阶段 D：失控与看门狗

| 测试 | 预期 |
|------|------|
| 有效行骤减（出赛道/全黑） | 断油、Disarm |
| 拔掉相机排线（武装态） | 100ms 级看门狗断油 |

---

## 5. 常见问题

| 现象 | 可能原因 |
|------|----------|
| 屏幕 assert / 卡死 | 横屏几何；确认 `set_dir` 在 `init` 前、菜单 30 列 |
| 上电电机转 | `DEBUG_NO_DRIVE=0` 且已 Arm；或 `motor_hw_init` 未执行 |
| 舵机反向 | 改 `SERVO_DIR` 或机械中位 |
| 编译 IMG 尺寸错误 | `config.h` 与 MT9V03X 驱动不一致 |
| 菜单 Load 无效 | Flash 版本不匹配；Restore Def 后重 Save |

---

## 6. 与 Git 分支

当前完整可烧录代码在分支 **`output-user-code-portrait`**：

- 根目录源码 = `output_user_code/`
- 学习文档 = `docs/steps/`

---

## 7. 完成定义

- [ ] ADS 编译 0 error
- [ ] 竖屏菜单正常、无 assert
- [ ] `DEBUG_NO_DRIVE=1` 下舵机与遥测正常
- [ ] 低速试跑可完成直道+弯道
- [ ] 图像失效与看门狗可断油

全部打勾后，本套固件可作为当前智能车主线版本使用。

---

## 8. 延伸阅读

- 仓库内 `PROJECT_SUMMARY.md`：架构摘要
- `docs/steps/step0`～`step5`：分模块原理
- `~/Developer/test_version_codex/docs/steps`：旧版含 FSM 的迁移教程（读时注意版本差异）
