# 第 2 步：图像流水线——从灰度图到转向误差

> **目标**：理解 `image_process()` 三步走，以及 `hybrid_track_extract()` 如何填 `track_info_t`。
> 对应文件：`image.c`、`image.h`、`hybrid_8n_longest_col/hybrid_track.c`

---

## 1. 入口与出口

```c
void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out);
```

| 输入 | 含义 |
|------|------|
| `img` | 相机灰度图，顶行在前 |
| `duty_now` | 上一帧实际下发的占空比，用于混合高/低速权重 |

| 输出 | 控制真正用到的 |
|------|----------------|
| `out->error` | **唯一**送入 PD 的标量 |
| `out->valid_rows` | 失控保护 |
| `out->both_lost_rows` | 全白/丢线判定 |
| 逐行 `left/right/mid` | 调试、将来可视化 |

---

## 2. 三步流水线

```text
(1) 阈值
      image_threshold>0 → 固定阈值
      否则 → otsu_threshold() 抽样直方图大津法
              ↓
(2) hybrid_track_extract()
      最长白列起种 + 八邻域双边跟踪 + 丢线预测
              ↓
(3) weighted_error()
      8 行带权重，按 duty_now 混合低/高速表
```

对应 `image.c` 191–201 行。

---

## 3. 大津法（`otsu_threshold`）

- 行/列按 `OTSU_ROW_STEP`、`OTSU_COL_STEP` 抽样，省算力
- 结果钳在 `[OTSU_THRESHOLD_MIN, OTSU_THRESHOLD_MAX]`
- 全黑/全白时大津会漂，钳制是安全兜底

**菜单调试**：在赛道阴影处若自动阈值不稳，设 `Threshold=128`（或实测值）切手动模式。

---

## 4. Hybrid 边线跟踪（概要）

文件：`hybrid_8n_longest_col/hybrid_track.c`

从**图像最底行**向上逐行：

1. **最长白列**作锚点，找白种子
2. 严格八邻域跟踪左右边
3. 边丢失 → 小窗口重捕获 → 仍失败则宽度表预测
4. 输出每行 `left/right/mid/width` 与 `left_lost/right_lost`

关键配置见 `config.h` 的 `HYBRID_*` 宏。  
`valid_rows` = 成功跟踪到的行数（从底向上）。

坐标：`out` 里 row=0 仍是**离车最近**的行；读相机数组时用 `RAW_ROW(row) = IMG_H - 1 - row`。

---

## 5. 加权误差（`weighted_error`）

```text
error = Σ w(row) · (mid[row] - IMG_CENTER) / Σ w(row)
```

权重来源：

- **行带**：8 带，每带 15 行；低速表近端重，高速表远端重
- **混合**：`k = duty_now / DUTY_HARD_CAP`（Q8 定点）在两张表间插值
- **置信缩放**：
  - 双边实测 → 100%
  - 单边重建 → `STEER_W_SINGLE_EDGE_PCT`（50%）
  - 双边丢失预测 → `STEER_W_BOTH_LOST_PCT`（0%，不参与）

`w_sum==0` 时 error=0，由 `image_track_invalid()` 触发失控保护。

---

## 6. 图像健康（`image_track_invalid`）

| 条件 | 结果 |
|------|------|
| `valid_rows < FAILSAFE_MIN_ROWS` | 失效；若=0 则 `severe=1` |
| 双边丢失比例 ≥ 70% | 失效 |
| 双边丢失比例 ≥ 90% | 严重失效（更快断油） |

在 `cpu0_main.c` 里累计帧数后断油，与 PD 无关。

---

## 7. 阅读顺序建议

1. `image.h` → 看 `track_info_t` 每个字段
2. `image.c` → `image_process()` 总入口
3. `hybrid_track.h` → 诊断结构体（可选）
4. `hybrid_track.c` → 从 `hybrid_track_extract()` 主循环读起（较长，可分多次）

---

## 8. 自查

- [ ] 能说出 `error` 是怎么从多行 `mid[]` 合成一个数的
- [ ] 知道 `duty_now` 为什么会影响权重（前瞻随速度）
- [ ] 知道 `both_lost` 行为什么不参与误差累加
- [ ] 知道 `RAW_ROW` 为什么存在（相机顶行 vs 逻辑底行）

下一步：[控制律](./step3_控制律.md)
