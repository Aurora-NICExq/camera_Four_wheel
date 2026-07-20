# 第 2 步：图像流水线——从灰度图到转向误差

> **目标**：理解 `image_process()` 四步走，以及 18th「最长白列 + 十字补线」如何填 `track_info_t`。
> 对应文件：`image.c`、`image.h`（算法移植自 the-18th-smartcar `Camera.c`）

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

## 2. 四步流水线

```text
(1) 阈值
      image_threshold>0 → 固定阈值
      否则 → otsu_threshold() 抽样直方图大津法
              ↓
(2) binarize()
      灰度 → 0/255 二值图 image_bin[][]
              ↓
(3) longest_white_column()
      统计每列白像素高度 → 找左右最长白列 → 白-黑-黑边线扫描
              ↓
(4) cross_detect()
      双边丢线足够多 → 找上下拐点 → left/right_add_line 或 lengthen 补线
              ↓
      export_track() + weighted_error()
```

对应 `image.c` 中 `image_process()`。

---

## 3. 大津法（`otsu_threshold`）

- 行/列按 `OTSU_ROW_STEP`、`OTSU_COL_STEP` 抽样，省算力
- 结果钳在 `[OTSU_THRESHOLD_MIN, OTSU_THRESHOLD_MAX]`
- 全黑/全白时大津会漂，钳制是安全兜底

**菜单调试**：在赛道阴影处若自动阈值不稳，设 `Threshold=128`（或实测值）切手动模式。

---

## 4. 最长白列边线（`longest_white_column`）

从图像**最底行**向上，在 `[TH18_COL_MARGIN, IMG_W - TH18_COL_MARGIN]` 内：

1. 统计每列连续白像素高度 → `white_column[]`
2. **左扫**找最长白列 → `longest_white_left_col`
3. **右扫**独立找最长白列 → `longest_white_right_col`
4. 在最长白列高度范围内，每行用 **白-黑-黑** 模板找左右边

边丢失时该行 `left_lost` / `right_lost` 置 1；`both_lost_time` 统计全图双边丢线行数。

坐标：`export_track()` 用 `TR_ROW(ir) = IMG_H - 1 - ir` 翻转，使 `track_info_t` 里 row=0 为**离车最近**的行。

---

## 5. 十字补线（`cross_detect`）

当 `both_lost_time >= TH18_CROSS_BOTH_LOST_MIN`：

1. `find_up_point`：从底向上找左右上拐点
2. `find_down_point`：从上拐点附近向下找下拐点
3. 根据四象限组合，调用 `left_add_line` / `right_add_line` 或 `lengthen_*_boundry` 补线
4. `mark_cross_fill` 标记补线区间 → `cross_filled[]`，供加权误差降权使用

`detect_cross()` 直接读 `ti->cross_valid`（由 `cross_detect` 写入）。

---

## 6. 加权误差（`weighted_error`）

```text
error = Σ w(row) · (mid[row] - IMG_CENTER) / Σ w(row)
```

权重来源：

- **行带**：8 带，每带 15 行；低速表近端重，高速表远端重
- **混合**：`k = duty_now / DUTY_HARD_CAP`（Q8 定点）在两张表间插值
- **置信缩放**：
  - 双边实测 → 100%
  - 单边重建 → `STEER_W_SINGLE_EDGE_PCT`（50%）
  - 十字补线 → `STEER_W_CROSS_FILL_PCT`（70%）
  - 双边丢失 → `STEER_W_BOTH_LOST_PCT`（0%，不参与）

`w_sum==0` 时 error=0，由 `image_track_invalid()` 触发失控保护。

---

## 7. 图像健康（`image_track_invalid`）

| 条件 | 结果 |
|------|------|
| `valid_rows < FAILSAFE_MIN_ROWS` | 失效；若=0 则 `severe=1` |
| 双边丢失比例 ≥ 70% | 失效 |
| 双边丢失比例 ≥ 90% | 严重失效（更快断油） |

在 `cpu0_main.c` 里累计帧数后断油，与 PD 无关。

---

## 8. 阅读顺序建议

1. `image.h` → 看 `track_info_t` 每个字段
2. `image.c` → `image_process()` 总入口
3. `longest_white_column()` → 基础边线
4. `cross_detect()` → 十字拐点与补线

---

## 9. 自查

- [ ] 能说出 `error` 是怎么从多行 `mid[]` 合成一个数的
- [ ] 知道 `duty_now` 为什么会影响权重（前瞻随速度）
- [ ] 知道 `both_lost` 行为什么不参与误差累加
- [ ] 知道 `TR_ROW` 为什么存在（相机顶行 vs 逻辑底行）

下一步：[控制律](./step3_控制律.md)
