# test_CarRun

TC264 双核智能车摄像头巡线固件。根目录为**唯一源码副本**——应用逻辑与逐飞库工程分离，在 Mac 上改码、push 后在烧录机 pull，用 ADS (Tasking) 编译烧录。

## 硬件

| 组件 | 型号 / 说明 |
|------|-------------|
| MCU | Infineon AURIX TC264（双核 TriCore） |
| 摄像头 | MT9V034，188×120 灰度 |
| 屏幕 | IPS200 SPI 彩屏（调试画面） |
| 执行器 | 舵机转向 + 双轮 PWM 驱动 |

引脚定义见 [`pins.h`](pins.h)。IPS200 背光本板接 `P20_14`（非逐飞默认 `P15_4`）。

## 控制链

```
MT9V034 帧 → image.c（二值化 + 边线 + 误差）
           → control.c（PD 转向 + 占空比）
           → motor.c（舵机 / 电机 PWM）
```

在线调参由 `menu.c` 提供，菜单项与变量绑定在 [`menu_config.c`](menu_config.c)。

## 双核分工

| 核 | 职责 | 入口 |
|----|------|------|
| CPU0 (1.6P) | 相机 DMA、图像处理、控制、电机 | [`cpu0_main.c`](cpu0_main.c) |
| CPU1 (1.6E) | 按键、菜单、IPS200 刷屏 | [`cpu1_main.c`](cpu1_main.c) |

跨核共享数据与所有权规则见 [`shared.h`](shared.h) 顶部注释（这份注释会随代码烧录，是唯一会到车上的架构说明）。

要点：

- 调参量（Kp/Kd、Threshold、Look Far 等）：CPU1 写，CPU0 读
- 电机与控制状态：CPU0 独占；CPU1 复位通过 `shared_ctrl_reset_req`
- 调试帧 `shared_disp`：单槽邮箱，CPU0 填、CPU1 画（仅在 Camera 页请求，避免每帧 22.5 KB 拷贝）
- CPU1 心跳超时（`CPU1_ALIVE_TIMEOUT_US`）触发与丢线相同的驱动锁死

## 巡线算法（`image.c`）

1. **二值化** — 固定阈值（菜单 Threshold=0 时走大津法 Otsu）
2. **边线提取** — 八邻域从底行向上爬线，得左右边界
3. **十字补线** — 向量法拐点检测 + 开口宽度校验，可选开启（菜单 Cross Fill）
4. **误差** — 单行前瞻：菜单 Look Far 指定瞄准行，该行双边丢线时向近端滑到第一条有效行；`aim_row=0` 触发丢线保护

常量与默认值在 [`config.h`](config.h)；可调运行时参数通过菜单修改。

## 目录结构

```
.
├── cpu0_main.c      # CPU0 主循环（实时链）
├── cpu1_main.c      # CPU1 主循环（人机界面）
├── shared.h / .c    # 双核共享数据
├── image.c / .h     # 图像处理与巡线
├── control.c / .h   # PD 控制
├── motor.c / .h     # PWM 驱动
├── menu.c / .h      # 菜单框架
├── menu_config.c    # 菜单项定义
├── menu_port.c / .h # 按键与屏幕端口
├── config.h         # 全局常量与预设
├── pins.h           # 引脚宏
├── isr.c            # 中断服务
├── isr_config.h     # 中断路由
└── docs/代码逐行注释/  # 各源文件逐行中文讲解
```

## 菜单速查

| 项 | 含义 |
|----|------|
| Armed | 发车使能（含延时与超时） |
| Kp / Kd / D Filt Alpha | 转向 PD 参数 |
| Threshold | 二值化阈值（0 = Otsu 自动） |
| Cross Fill | 十字路口补线开关 |
| Look Far | 前瞻瞄准行（越大越远） |
| Duty | 直行占空比 |
| Stop Time | Armed 超时（秒） |
| Lost Fr | 连续丢线帧数阈值（触发锁死） |
| Race Preset | 加载比赛预设 |
| Camera | 实时灰度图 + 边线叠加调试 |

按键：UP/DOWN 移动，ENTER 进入/确认，BACK 返回。长按 ENTER 切换 Fine Step 微调步长。

## 编译与烧录

本仓库**不含**逐飞库与 ADS 工程文件，仅存放应用层 `.c/.h`。

1. Mac 侧修改源码，`git push`
2. 烧录机 `git pull`
3. 在逐飞 TC264 工程（ADS）中编译、下载

编译前确认 `config.h` 中 `IMG_W`/`IMG_H` 与 `MT9V03X_W`/`MT9V03X_H` 一致（`motor.c` 有 `#error` 守卫）。

## 安全机制

- **丢线保护** — `aim_row == 0` 连续超过 Lost Fr 帧后锁死驱动，须撤销 Armed 才能恢复
- **Armed 超时** — Stop Time 到期自动停车
- **CPU1 看门狗** — CPU1 主循环心跳停止超过 500 ms 同样锁死驱动

## 开发约定

- 与挂钟相关的逻辑一律用 `hal_time_us()`，不用帧计数
- 新增影响控制输出的机制须可观测（进遥测或 Camera 调试页）
- 调参优先删无效机制、修上游，再考虑加新旋钮

更细的逐行说明见 [`docs/代码逐行注释/`](docs/代码逐行注释/README.md)。
