# 大津法（Otsu）二值化阈值 — 代码讲解

本文说明工程里**哪一段实现了大津法**、数学在做什么、以及和流水线其它部分怎么衔接。

> 本地说明文档，已由 `.gitignore` 的 `*.md` 排除，不进入 git。

---

## 1. 文件位置（结论先看）

| 角色 | 路径 | 函数 |
|------|------|------|
| **车端 / 当前工作区** | `image.c` | `static uint8_t otsu_threshold(...)`（约第 18–67 行） |
| 相关配置 | `config.h` | `OTSU_*`、`FIXED_THRESHOLD` |
| 调用入口 | `image.c` → `image_process()` | `image_threshold==0` 时调用大津 |
| **上位机测试树** | `…/TC264_Hybrid_Tracking_Test/user/image.c` | 同名函数（实现更细：`i+1` 对齐、`OTSU_VAR_32BIT`） |

二值判定宏（阈值算出来之后用）：

```c
#define IS_WHITE(px, th) ((px) >= (th))   /* ≥ 阈值为白（赛道） */
```

大津**只负责求出一个 `uint8_t` 阈值**；真正“变成黑白图”是后续搜边里用 `IS_WHITE` 逐像素判断，并没有单独再生成一张二值图。

---

## 2. 在流水线里的位置

```
灰度图 img
    │
    ▼
┌─────────────────┐
│  otsu_threshold │  ← 本文件讲解的部分
│  或 固定阈值     │     （菜单 image_threshold > 0）
└────────┬────────┘
         │ out->threshold
         ▼
最长白列 → 逐行搜边 → 丢线重建 → 检测器 / 误差
（全程用同一个 threshold + IS_WHITE）
```

车端调用（`image_process`）：

```c
out->threshold = (image_threshold > 0) ? (uint8_t)image_threshold
                                       : otsu_threshold(img);
```

- `image_threshold == 0`（默认）：自动大津  
- `image_threshold > 0`：菜单手动固定阈值，**整帧跳过大津**

---

## 3. 大津法在干什么（直觉）

灰度图里通常有两类像素：偏暗（背景/黑线）和偏亮（赛道白区）。  
大津法在 `0…255` 里试每一个候选分割点 `t`，把直方图切成：

- 类 0：灰度 `≤ t`（或实现里累积到 `i` 的一侧）  
- 类 1：其余  

使得**类间方差**最大的那个 `t`，就是“最能把两类分开”的阈值。

标准形式（连续写法）：

\[
\sigma_B^2(t) = w_0(t)\,w_1(t)\,\bigl(\mu_0(t)-\mu_1(t)\bigr)^2
\]

其中 \(w_0,w_1\) 是两类像素占比，\(\mu_0,\mu_1\) 是两类灰度均值。  
实现里**不除以总像素数平方**（对所有候选 `t` 都是同一常数），只比较相对大小即可。

---

## 4. 车端实现逐步拆解（`test_CarRun/image.c`）

对应源码：

```c
static uint8_t otsu_threshold(const uint8_t img[IMG_H][IMG_W])
{
    uint32_t hist[256] = {0};
    uint32_t total = 0;
    /* ... */
}
```

### 4.1 抽样建直方图

```c
for (r = 0; r < IMG_H; r += OTSU_ROW_STEP)
    for (c = 0; c < IMG_W; c += OTSU_COL_STEP)
    {
        hist[img[r][c]]++;
        total++;
    }
```

- `hist[g]`：灰度值为 `g` 的**抽样**像素个数  
- `OTSU_ROW_STEP` / `OTSU_COL_STEP` 在 `config.h` 里默认都是 **2** → 大约只用 1/4 像素  
- 阈值是**全局统计量**，稀疏抽样通常对结果几乎无影响，却明显省时间  

注意：这里扫的是 `img[r][c]` 的**相机原始行序**（顶行 `r=0`），和后面逻辑行 `RAW_ROW` 翻转无关——大津不关心“近/远”，只关心灰度分布。

### 4.2 全图灰度和

```c
uint64_t sum_all = 0;
for (i = 0; i < 256; i++)
    sum_all += (uint64_t)i * hist[i];
```

即 \(\sum g\cdot h(g)\)，后面算两类均值要用。

### 4.3 扫一遍灰度级，找最大类间方差

```c
uint64_t best_var = 0;
uint16_t best_th  = FIXED_THRESHOLD;  /* 退化时的兜底 */
uint32_t w0 = 0;
uint64_t sum0 = 0;

for (i = 0; i < 256; i++)
{
    w0 += hist[i];
    if (w0 == 0) continue;           /* 左侧还是空类 */
    uint32_t w1 = total - w0;
    if (w1 == 0) break;              /* 右侧已空，再往上没意义 */
    sum0 += (uint64_t)i * hist[i];

    /* 等价于比较 w0*w1*(mu0-mu1)^2，避免先算浮点均值 */
    int64_t diff = (int64_t)(sum0 * w1) - (int64_t)((sum_all - sum0) * w0);
    uint64_t d2  = (uint64_t)((diff < 0) ? -diff : diff);
    uint64_t var = (d2 / w0) * (d2 / w1);

    if (var > best_var)
    {
        best_var = var;
        best_th  = i;
    }
}
```

符号对应：

| 代码 | 含义 |
|------|------|
| `i` | 当前候选：把灰度 `0…i` 当作一类（实现里的“黑侧”累积） |
| `w0` | 该类像素数 |
| `w1` | 另一类像素数 |
| `sum0` | 该类灰度和 |
| `sum_all - sum0` | 另一类灰度和 |
| `diff` | 与 \(w_0 w_1(\mu_0-\mu_1)\) 成比例的整数核心 |
| `var` | 与类间方差成比例的量（先除后乘，防溢出，阈值级误差通常 &lt; 1） |

全程**整数**，无 `float`，保证车端与 PC 回放行为一致。

### 4.4 钳制到安全区间

```c
if (best_th < OTSU_THRESHOLD_MIN) best_th = OTSU_THRESHOLD_MIN;  /* 默认 40 */
if (best_th > OTSU_THRESHOLD_MAX) best_th = OTSU_THRESHOLD_MAX;  /* 默认 200 */
return (uint8_t)best_th;
```

全黑/全白、强曝光时，直方图几乎单峰，大津会漂到荒谬值；用 `[40, 200]` 兜底，避免整帧“全白”或“全黑”误判。

`FIXED_THRESHOLD`（128）只在**从未更新过 `best_var`** 的退化情况下作为初值。

---

## 5. 相关配置（`config.h`）

```c
#define FIXED_THRESHOLD         (128)   /* 大津兜底 / 手动参考 */
#define OTSU_ROW_STEP           (2)     /* 直方图行抽样 */
#define OTSU_COL_STEP           (2)     /* 直方图列抽样 */
#define OTSU_THRESHOLD_MIN      (40)    /* 大津下限保护 */
#define OTSU_THRESHOLD_MAX      (200)   /* 大津上限保护 */
```

菜单项 `Threshold` 绑定的是 `volatile int16_t image_threshold`（定义在 `image.c`）：

- `0` → 走 `otsu_threshold`  
- `1…255` → 固定阈值，不跑大津  

---

## 6. 和上位机版本的差异（了解即可）

上位机编译的  
`TC264_Hybrid_Tracking_Test/user/image.c` 里同名函数逻辑相同，额外两点更“抠”：

1. **`best_th++`**  
   循环里 `i` 表示“黑类包含到灰度 `i`”；而 `IS_WHITE` 是 `px >= th`。  
   找到最优分割后把阈值 `+1`，让“类边界”和 `>=` 语义对齐。

2. **`OTSU_VAR_32BIT`**  
   在样本量足够小时，方差路径用 32 位除法（MCU 上比 64 位软除更便宜），阈值结果仍与 64 位路径一致。

车端当前工作区版本**没有**做 `best_th++`；若要对齐上位机语义，需要两边统一这一处。

---

## 7. 常见现象对照

| 现象 | 可能原因 |
|------|----------|
| 光线变化时边线仍稳 | 大津每帧自适应阈值 |
| 突然全白/全黑误跟 | 阈值顶到 MIN/MAX，或直方图单峰；可看 `out->threshold` |
| 菜单改 Threshold 立即变 | 走了固定阈值分支，大津被绕过 |
| 想加速 | 已抽样 1/4；再加大 `OTSU_*_STEP` 收益有限，优先别动算法热路径 |

---

## 8. 一句话总结

**大津法 = `image.c` 里的 `otsu_threshold()`：**  
抽样直方图 → 扫灰度找最大类间方差 → 钳制到 `[OTSU_THRESHOLD_MIN, OTSU_THRESHOLD_MAX]` → 交给 `image_process` 写成 `out->threshold`，后续最长白列/搜边全部用 `IS_WHITE(px, threshold)` 解释黑白。
