# image.h 逐行注释

> 行号与源文件一致。

---

```c
/* image.h */                                     // L1:  文件说明
#ifndef IMAGE_H                                   // L2:  头文件保护开始
#define IMAGE_H                                   // L3:  定义 IMAGE_H 宏

#include <stdint.h>                               // L5:  标准整数类型 uint8_t 等
#include "config.h"                               // L6:  IMG_H、IMG_W、IMG_CENTER 等尺寸常量

// ─────────────── track_info_t 巡线输出结构 ───────────────

typedef struct                                    // L8:  每帧 image_process 填写的轨迹与误差
{
    // ── 按 track 行索引的边界数组（长度 IMG_H=120）──
    // track 行号 tr：0=近车（图像底部），119=远（图像顶部）
    // 与 image.c 中 TR_ROW(ir) 一致：tr = IMG_H - 1 - ir

    uint8_t  left [IMG_H];                        // L10: 每行左边界列号 [0, IMG_W-1]
    uint8_t  right[IMG_H];                        // L11: 每行右边界列号
    uint8_t  mid  [IMG_H];                        // L12: 每行中线列号，通常 (left+right)/2
    uint8_t  left_lost [IMG_H];                   // L13: 左丢线标志；1=该行左边界不可信
    uint8_t  right_lost[IMG_H];                   // L14: 右丢线标志；1=该行右边界不可信

    // ── 本帧标量输出 ──

    int16_t  error;                               // L16: 转向误差 = look_ahead_error 返回值
                                                  //      正值=赛道中心在图像右侧（需右转）
    uint8_t  err_hold;                            // L17: 前瞻全丢时已连续保持误差的帧数
                                                  //      对应 g_hold_frames，供遥测观测
    uint8_t  look_rows;                           // L18: 前瞻窗口内实际参与平均的行数 n
                                                  //      n=0 时 error 为保持/衰减值而非新算
    uint8_t  both_lost_rows;                      // L19: 有效搜索区内左右同时丢线的行数
    uint8_t  threshold;                           // L20: 本帧二值化阈值（Otsu 或菜单手动值）

    // ── 十字补线元数据（cross_fill 写入）──

    uint8_t  cross_filled[IMG_H];                 // L22: 按 track 行：1=该行边界被补线改写
                                                  //      image_debug_show 用黄色区分
    uint8_t  cross_valid;                         // L23: 本帧是否确认并执行了十字补线；1=是
    uint8_t  cross_lo;                            // L24: 补线区 track 下界（近端，较大 tr 侧）
                                                  //      与 cross_hi 一起框定补线影响范围
                                                  //      未补线时保持 init_cross_meta 的 0
    uint8_t  cross_hi;                            // L25: 补线区 track 上界（远端，较小 tr 侧）
                                                  //      mark_cross_fill_rows 首次标记时设置
    uint8_t  inflect_row;                         // L26: 左十字拐点对应的 track 行号
                                                  //      0xFF=本帧无有效拐点；供遥测/调试
} track_info_t;

// ─────────────── 菜单可调全局变量 ───────────────

extern volatile int16_t image_threshold;          // L29: 二值化阈值；>0 强制使用，0=自动 Otsu
extern volatile uint8_t image_cross_fill;         // L30: 十字补线开关；0=关闭 cross_fill
extern volatile uint16_t steer_look_far;          // L31: 前瞻起点（菜单 Look Far）
                                                  //      默认 STEER_LOOK_FAR_DEFAULT=115
                                                  //      look_ahead_error 从此向近端取 20 行

// ─────────────── 对外接口 ───────────────

void image_process(const uint8_t img[IMG_H][IMG_W], track_info_t *out);
                                                  // L33: 每帧主入口：二值化→巡线→补线→误差
                                                  //      不再接收 duty；无隐式速度增益调度
void image_debug_show(const track_info_t *ti);    // L34: IPS200 上叠加二值图与左/右/中线
                                                  //      补线段黄色，正常左蓝右红，中绿

#endif /* IMAGE_H */                              // L36: 头文件保护结束
```

---

## 字段关系速查

| 字段 | 写入者 | 含义 |
|------|--------|------|
| `left/right/mid` | `export_track` | 每 track 行的赛道几何 |
| `left_lost/right_lost` | `export_track` | 边界是否贴在图像边缘（丢线） |
| `error` | `look_ahead_error` | 控制环 PD 的输入偏差 |
| `look_rows` | `look_ahead_error` | 前瞻实际样本数；`n<span` 表示部分行被跳过 |
| `err_hold` | `image_process` | 全丢前瞻时的保持计数 |
| `both_lost_rows` | `export_track` | 双丢行统计 |
| `threshold` | `image_process` | 本帧二值阈值 |
| `cross_filled[]` | `mark_cross_fill_rows` | 逐行补线标记（调试色） |
| `cross_valid` | `cross_fill` | 是否真十字且已补线 |
| `cross_lo` / `cross_hi` | `mark_cross_fill_rows` | 补线区在 track 行轴上的范围 |
| `inflect_row` | `cross_fill` | 拐点 track 行；`0xFF` 无效 |

---

*行号与 `image.h` 一一对应（当前 37 行）。若源码增删行，请以 git 版本为准核对。*
