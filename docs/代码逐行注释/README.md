# 固件源码逐行注释

本目录收录根目录各 `.c` / `.h` 源文件的逐行中文讲解，行号与源码一一对应。

## 文件索引

| 文档 | 源文件 | 说明 |
|------|--------|------|
| [config.h.md](config.h.md) | `config.h` | 图像尺寸、前瞻、八邻域、十字补线、舵机、PD、预设参数 |
| [image.c.md](image.c.md) | `image.c` | 八邻域巡线、十字补线、前瞻误差 |
| [image.h.md](image.h.md) | `image.h` | `track_info_t` 与图像模块接口 |
| [control.c.md](control.c.md) | `control.c` | PD 转向与占空比软启动 |
| [control.h.md](control.h.md) | `control.h` | 控制输出结构与全局参数 |
| [motor.c.md](motor.c.md) | `motor.c` | 舵机/电机 PWM 驱动 |
| [motor.h.md](motor.h.md) | `motor.h` | 电机模块接口 |
| [cpu0_main.c.md](cpu0_main.c.md) | `cpu0_main.c` | 主循环：图像→控制→电机、丢线保护、发车计时 |
| [cpu1_main.c.md](cpu1_main.c.md) | `cpu1_main.c` | CPU1 占位核 |
| [menu.c.md](menu.c.md) | `menu.c` | 菜单 FSM、Race Preset |
| [menu.h.md](menu.h.md) | `menu.h` | 菜单类型与宏 |
| [menu_config.c.md](menu_config.c.md) | `menu_config.c` | 菜单项与变量绑定 |
| [menu_port.c.md](menu_port.c.md) | `menu_port.c` | GPIO 按键、IPS200 显示端口 |
| [menu_port.h.md](menu_port.h.md) | `menu_port.h` | 菜单端口接口 |
| [isr.c.md](isr.c.md) | `isr.c` | 中断服务例程 |
| [isr_config.h.md](isr_config.h.md) | `isr_config.h` | 中断路由与优先级 |
| [pins.h.md](pins.h.md) | `pins.h` | 引脚宏定义 |

## 阅读建议

1. 从 `cpu0_main.c.md` 了解主循环数据流。
2. 读 `image.c.md` 理解巡线与十字补线（当前为方向序列 `4,4,6,6,6` + 开口宽度校验版本）。
3. 读 `control.c.md` 理解 PD 如何映射到舵机 PWM。
