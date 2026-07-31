# menu.c 逐行注释

> 行号与源文件一致。

菜单核心逻辑：三层导航 FSM（`NAV_LIST` 列表 / `NAV_EDIT` 数值编辑 / `NAV_PRESET` 赛道预设）、滚动列表绘制、测试模式覆盖（摄像头/电机/左轮）。Race Preset 先 `apply_defaults()` 再覆盖 Low/Mid/High 九元组参数。

## 导航 FSM

```
                    ┌─────────────┐
         BACK       │  NAV_LIST   │ ENTER(数值项)
    ┌────────────────┤  主列表     ├──────────────┐
    │                └──────┬──────┘              │
    │ ENTER(Race Preset)    │                     ▼
    │                       │              ┌─────────────┐
    ▼                       │              │  NAV_EDIT   │
┌─────────────┐              │              │  工作副本   │
│ NAV_PRESET  │◄─────────────┘              │ s_edit_val  │
│ 三档预设    │  ENTER 应用后回 LIST         └─────────────┘
└─────────────┘

并行覆盖态（非 nav 枚举）：s_camera_view / s_align_test_mode /
s_motor_test_mode / s_left_test_mode — 按键仅 BACK 退出。
```

## Race Preset 三档（config.h）

| 档位 | Kp | Kd | Look Far | Duty | 说明 |
|------|-----|-----|----------|------|------|
| Low | 1.50 | 1.20 | 71 | 2700 | 已实测 |
| Mid | 1.55 | 1.15 | 79 | 3000 | 已实测 |
| High | KP | KD | 默认 | STRAIGHT_DUTY | 等同 Restore Def 默认 |

```c
// L1: 文件头注释
/* menu.c */
// L2: 菜单类型与对外 API
#include "menu.h"
// L3: IPS200 与 GPIO 键端口
#include "menu_port.h"
// L4: drive_armed、drive_timed_out、control_init
#include "control.h"
// L5: 图像调参变量 extern
#include "image.h"
// L6: motor_stop、motor_reset
#include "motor.h"
// L7: 空行

// L8: steer_kp 定义在 control.c，此处仅引用
extern volatile float    steer_kp;
// L9: Fine Step 全局开关，定义在本文件
volatile uint8_t menu_fine_step = 0;
// L10: 空行

// L11: 其余转向/图像调参变量 extern
extern volatile float    steer_kd;
// L12: D 项滤波 alpha
extern volatile float    steer_d_filt_alpha;
// L13: 二值化阈值
extern volatile int16_t  image_threshold;
// L14: 十字补线
extern volatile uint8_t  image_cross_fill;
// L15: 前瞻起点行 Look Far
extern volatile uint16_t steer_look_far;
// L16: 空行

// L17: 内容区行数 = 总行 - 标题行
#define CONTENT_ROWS  (MENU_ROWS - 1)          // 第 0 行为标题
// L18: 数值列起始字符列（竖屏 30 列布局）
#define VALUE_COL     (18)                     // 竖屏 30 列：左侧标签，右侧数值
// L19: 数值区宽度
#define VALUE_WIDTH   (MENU_COLS - VALUE_COL)
// L20: 长按重复时步长 ×10
#define KEY_STEP_MULT (10.0f)                   // 长按自动重复步长倍率
// L21: Fine Step 开启时步长 ×0.1
#define FINE_STEP_DIV  (0.1f)                   // Fine Step 开启时步长倍率
// L22: 浮点最细步长，与 2 位小数显示一致
#define FINE_MIN_FLOAT (0.01f)                  // 浮点最小步长(与 2 位小数显示精度对齐)
// L23: 整型最细步长为 1，再小取整无效
#define FINE_MIN_INT   (1.0f)                   // 整型最小步长:再小会被取整吃掉,按键像失灵
// L24: 空行

// L25: 导航状态枚举：列表 / 编辑 / 预设子页
typedef enum { NAV_LIST, NAV_EDIT, NAV_PRESET } nav_state_e;
// L26: 空行

// L27: 预设档位数 Low / Mid / High
#define PRESET_COUNT (3)
// L28: 空行

// L29: 单档预设数据结构
typedef struct
// L30: 左花括号
{
// L31: 显示名
    const char *name;
// L32: 转向 Kp
    float    kp;
// L33: 转向 Kd
    float    kd;
// L34: D 滤波 alpha
    float    d_alpha;
// L35: 图像阈值
    int16_t  threshold;
// L36: 十字补线
    uint8_t  cross_fill;
// L37: Look Far
    uint16_t look_far;
// L38: 直行 duty
    uint16_t duty;
// L39: Armed 超时秒数
    uint16_t stop_time;
// L40: 结构体结束
} preset_t;
// L41: 空行

// L42: 三档常量表，数值来自 config.h PRESET_* 宏
static const preset_t s_presets[PRESET_COUNT] = {
// L43: 低速档
    { "Low  (tested)",
// L44: Kp
      PRESET_LOW_KP, PRESET_LOW_KD,
// L45: D alpha
      PRESET_LOW_D_ALPHA,
// L46: 阈值与十字
      PRESET_LOW_THRESHOLD, PRESET_LOW_CROSS_FILL,
// L47: Look Far、duty、停车时间
      PRESET_LOW_LOOK_FAR, PRESET_LOW_DUTY, PRESET_LOW_STOP_TIME },
// L48: 中速档
    { "Mid  (tested)",
// L49: Kp Kd
      PRESET_MID_KP, PRESET_MID_KD,
// L50: D alpha
      PRESET_MID_D_ALPHA,
// L51: 阈值十字
      PRESET_MID_THRESHOLD, PRESET_MID_CROSS_FILL,
// L52: Look Far duty stop
      PRESET_MID_LOOK_FAR, PRESET_MID_DUTY, PRESET_MID_STOP_TIME },
// L53: 高速档（等于 config 默认）
    { "High (= Default)",
// L54: Kp Kd 引用 KP KD
      PRESET_HIGH_KP, PRESET_HIGH_KD,
// L55: D alpha
      PRESET_HIGH_D_ALPHA,
// L56: 阈值十字
      PRESET_HIGH_THRESHOLD, PRESET_HIGH_CROSS_FILL,
// L57: 默认 Look Far、duty、超时
      PRESET_HIGH_LOOK_FAR, PRESET_HIGH_DUTY, PRESET_HIGH_STOP_TIME },
// L58: 表结束
};
// L59: 空行

// L60: 当前导航状态，初始 NAV_LIST
static nav_state_e       s_nav;
// L61: 摄像头全屏模式标志
static uint8_t           s_camera_view;
// L62: 对齐测试（摄像头+停车）
static uint8_t           s_align_test_mode;
// L63: 双电机 20% 测试
static uint8_t           s_motor_test_mode;
// L64: 仅左 PWM 测试
static uint8_t           s_left_test_mode;
// L65: 预设页光标 0..2
static uint8_t           s_preset_cursor;       // 预设子页面选中档位
// L66: 主列表光标索引
static uint8_t           s_cursor;              // 选中项索引
// L67: 滚动窗口顶部项索引
static uint8_t           s_top;                 // 滚动窗口顶
// L68: 编辑态工作副本，确认后才写回变量
static float             s_edit_val;            // 编辑工作副本
// L69: 当前正在编辑的菜单项指针
static const menu_item_t *s_edit_item;
// L70: 空行

// L71: 四舍五入 float→int32
static int32_t round_f(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }
// L72: 空行

// L73: 从绑定变量读取当前值为 float
static float item_get(const menu_item_t *it)
// L74: 左花括号
{
// L75: 按类型分发
    switch (it->type)
// L76: 左花括号
    {
// L77: int16 强转 float
    case ITEM_INT16:  return (float)(*(volatile int16_t  *)it->var);
// L78: uint16
    case ITEM_UINT16: return (float)(*(volatile uint16_t *)it->var);
// L79: float 直接读
    case ITEM_FLOAT:  return *(volatile float *)it->var;
// L80: bool 转 0/1
    case ITEM_BOOL:   return (*(volatile uint8_t *)it->var) ? 1.0f : 0.0f;
// L81: 其它返回 0
    default:          return 0.0f;
// L82: switch 结束
    }
// L83: 函数结束
}
// L84: 空行

// L85: 将 float 值钳位后写回绑定变量
static void item_set(const menu_item_t *it, float v)
// L86: 左花括号
{
// L87: 下限钳位
    if (v < it->min) v = it->min;
// L88: 上限钳位
    if (v > it->max) v = it->max;
// L89: 按类型写回
    switch (it->type)
// L90: 左花括号
    {
// L91: int16 四舍五入写入
    case ITEM_INT16:  *(volatile int16_t  *)it->var = (int16_t)round_f(v);   break;
// L92: uint16
    case ITEM_UINT16: *(volatile uint16_t *)it->var = (uint16_t)round_f(v);  break;
// L93: float 原样写
    case ITEM_FLOAT:  *(volatile float    *)it->var = v;                     break;
// L94: bool 非零即 1
    case ITEM_BOOL:   *(volatile uint8_t  *)it->var = (v != 0.0f) ? 1u : 0u; break;
// L95: ACTION 等无操作
    default: break;
// L96: switch 结束
    }
// L97: 函数结束
}
// L98: 空行

// L99: 将 name 左对齐填入 MENU_COLS 宽空格缓冲区
static void build_label(char *dst, const char *name)
// L100: 左花括号
{
// L101: 循环变量
    uint8_t i;
// L102: 先填满空格
    for (i = 0; i < MENU_COLS; i++) dst[i] = ' ';
// L103: C 字符串结尾
    dst[MENU_COLS] = '\0';
// L104: 若有 name 则逐字覆盖左侧
    if (name) for (i = 0; i < MENU_COLS && name[i]; i++) dst[i] = name[i];
// L105: 函数结束
}
// L106: 空行

// L107: 绘制顶栏标题，格式 "= 文字 ="
static void draw_title(const char *txt)
// L108: 左花括号
{
// L109: 标题缓冲
    char t[MENU_COLS + 1];
// L110: 循环变量 i,j
    uint8_t i, j;
// L111: 初始全空格
    for (i = 0; i < MENU_COLS; i++) t[i] = ' ';
// L112: 结尾符
    t[MENU_COLS] = '\0';
// L113: 左装饰 "= "
    t[0] = '='; t[1] = ' '; j = 2;
// L114: 拷贝标题文字
    for (i = 0; txt && txt[i] && j < MENU_COLS - 2; i++) t[j++] = txt[i];
// L115: 右侧补 " ="
    if (j < MENU_COLS - 1) { t[j++] = ' '; t[j++] = '='; }
// L116: 第 0 行 TITLE 样式
    menu_port_draw_text(0, 0, t, MENU_STYLE_TITLE);
// L117: 函数结束
}
// L118: 空行

// L119: 编辑模式标题：前缀 [F] 或 [E] + 项名
static void draw_title_edit(const char *name)
// L120: 左花括号
{
// L121: 缓冲
    char t[MENU_COLS + 1];
// L122: Fine Step 开显示 [F]，否则 [E]（Edit）
    const char *pfx = menu_fine_step ? "[F] " : "[E] ";
// L123: i,j
    uint8_t i, j = 0;
// L124: 填空格
    for (i = 0; i < MENU_COLS; i++) t[i] = ' ';
// L125: 结尾
    t[MENU_COLS] = '\0';
// L126: 写前缀
    for (i = 0; pfx[i] && j < MENU_COLS; i++) t[j++] = pfx[i];
// L127: 写项名
    for (i = 0; name && name[i] && j < MENU_COLS; i++) t[j++] = name[i];
// L128: EDIT 样式绘第 0 行
    menu_port_draw_text(0, 0, t, MENU_STYLE_EDIT);
// L129: 函数结束
}
// L130: 空行

// L131: 在指定行绘制数值列
static void draw_value(const menu_item_t *it, uint8_t row, float v, menu_style_e st)
// L132: 左花括号
{
// L133: 按类型选择绘制函数
    switch (it->type)
// L134: 左花括号
    {
// L135: 浮点 4 位整数 2 位小数
    case ITEM_FLOAT:  menu_port_draw_float(VALUE_COL, row, v, 4, 2, st);                       break;
// L136: 有符号整型
    case ITEM_INT16:  menu_port_draw_int (VALUE_COL, row, (int32_t)round_f(v), VALUE_WIDTH, st); break;
// L137: 无符号整型
    case ITEM_UINT16: menu_port_draw_uint(VALUE_COL, row, (uint32_t)round_f(v), VALUE_WIDTH, st);break;
// L138: 布尔：特殊显示 Armed 超时
    case ITEM_BOOL:
// L139: 若武装已超时且显示为 ON，显示 TMO 而非 ON
      if (drive_timed_out && v != 0.0f)
// L140: 超时提示
        menu_port_draw_text(VALUE_COL, row, "TMO", st);
// L141: 否则 ON/OFF
      else
        menu_port_draw_text(VALUE_COL, row, (v != 0.0f) ? "ON " : "OFF", st);
// L142: break bool
      break;
// L143: 默认无绘制
    default: break;
// L144: switch 结束
    }
// L145: 函数结束
}
// L146: 空行

// L147: 绘制一行菜单项：标签 + 可选数值
static void draw_item_row(uint8_t index, uint8_t screen_row, bool selected, bool editing)
// L148: 左花括号
{
// L149: 取表项
    const menu_item_t *it = &menu_items[index];
// L150: 标签缓冲
    char label[MENU_COLS + 1];
// L151: 样式：编辑 > 选中 > 普通
    menu_style_e st = editing ? MENU_STYLE_EDIT : (selected ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
// L152: 构建左对齐标签
    build_label(label, it->name);
// L153: 选中或编辑时行首加 "> "
    if (selected || editing) { label[0] = '>'; label[1] = ' '; }
// L154: 绘标签
    menu_port_draw_text(0, screen_row, label, st);
// L155: 非 ACTION 绘数值；编辑态用 s_edit_val
    if (it->type != ITEM_ACTION) draw_value(it, screen_row, editing ? s_edit_val : item_get(it), st);
// L156: 函数结束
}
// L157: 空行

// L158: 全量重绘列表（滚动窗口变化或初始化时）
static void draw_list_full(void)
// L159: 左花括号
{
// L160: 空白行缓冲
    char blank[MENU_COLS + 1];
// L161: 屏内行号 r
    uint8_t r;
// L162: 清屏
    menu_port_clear();
// L163: 标题 TUNING
    draw_title("TUNING");
// L164: 空白行模板
    build_label(blank, "");
// L165: 绘制 CONTENT_ROWS 行内容
    for (r = 0; r < CONTENT_ROWS; r++)
// L166: 左花括号
    {
// L167: 对应菜单项全局索引
        uint8_t i = (uint8_t)(s_top + r);
// L168: 有效项则绘制
        if (i < menu_item_count)
// L169: 屏行 r+1（0 为标题）；是否光标/编辑
            draw_item_row(i, (uint8_t)(r + 1), (i == s_cursor), (s_nav == NAV_EDIT && i == s_cursor));
// L170: 超出项数则填空行
        else
            menu_port_draw_text(0, (uint8_t)(r + 1), blank, MENU_STYLE_NORMAL);
// L171: for 结束
    }
// L172: 函数结束
}
// L173: 空行

// L174: 仅重绘可见行数值（Restore Def 后避免全屏刷新）
static void redraw_visible_values(void)
// L175: 左花括号
{
// L176: 行号
    uint8_t r;
// L177: 遍历可见区
    for (r = 0; r < CONTENT_ROWS; r++)
// L178: 左花括号
    {
// L179: 菜单项索引
        uint8_t i = (uint8_t)(s_top + r);
// L180: 无更多项则停
        if (i >= menu_item_count) break;
// L181: ACTION 无数值跳过
        if (menu_items[i].type != ITEM_ACTION)
// L182: 重绘数值列
            draw_value(&menu_items[i], (uint8_t)(r + 1), item_get(&menu_items[i]),
// L183: 光标行用 SELECTED 样式（续上行）
                       (i == s_cursor) ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
// L184: for 结束
    }
// L185: 函数结束
}
// L186: 空行

// L187: 列表上下移动光标，维护滚动窗口 s_top
static void list_move(int8_t dir)
// L188: 左花括号
{
// L189: 保存旧光标与窗口顶
    uint8_t old = s_cursor, old_top = s_top;
// L190: 上移：已在 0 则返回
    if (dir < 0) { if (s_cursor == 0) return; s_cursor--; }
// L191: 下移：已在末尾则返回
    else         { if (s_cursor + 1 >= menu_item_count) return; s_cursor++; }
// L192: 空行

// L193: 光标移到窗口上方：窗口顶上移
    if (s_cursor < s_top)                         s_top = s_cursor;
// L194: 光标移到窗口下方：窗口顶下移
    else if (s_cursor >= s_top + CONTENT_ROWS)    s_top = (uint8_t)(s_cursor - CONTENT_ROWS + 1);
// L195: 空行

// L196: 窗口滚动则需全量重绘
    if (s_top != old_top) { draw_list_full(); return; }
// L197: 否则只重绘旧行与新行（增量刷新）
    draw_item_row(old,      (uint8_t)((old - s_top) + 1),      false, false);
// L198: 新光标行高亮
    draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true,  false);
// L199: 函数结束
}
// L200: 空行

// L201: ENTER 处理：ACTION 回调 / BOOL 翻转 / 数值进编辑
static void item_enter(void)
// L202: 左花括号
{
// L203: 当前项
    const menu_item_t *it = &menu_items[s_cursor];
// L204: ACTION：直接调 action 并返回
    if (it->type == ITEM_ACTION) { if (it->action) it->action(); return; }
// L205: BOOL：原地 toggle 写回
    if (it->type == ITEM_BOOL)
// L206: 左花括号
    {
// L207: 0↔1 翻转并 commit
        item_set(it, (item_get(it) != 0.0f) ? 0.0f : 1.0f);   // 原子 toggle + commit
// L208: 重绘当前行
        draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true, false);
// L209: 返回
        return;
// L210: if BOOL 结束
    }
// L211: 数值项：进入 NAV_EDIT
    s_nav = NAV_EDIT;                                          // 数值 → 进入编辑（工作副本）
// L212: 记录编辑项
    s_edit_item = it;
// L213: 拷贝当前值到工作副本
    s_edit_val  = item_get(it);
// L214: 更新标题为编辑态
    draw_title_edit(it->name);
// L215: 重绘当前行为编辑样式
    draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true, true);
// L216: 函数结束
}
// L217: 空行

// L218: 编辑态 UP/DOWN 调整 s_edit_val
static void edit_adjust(int8_t dir, uint8_t repeat)
// L219: 左花括号
{
// L220: 当前编辑项
    const menu_item_t *it = s_edit_item;
// L221: 基础步长来自菜单表
    float step = it->step;
// L222: 空行

// L223: Fine Step 模式
    if (menu_fine_step)
// L224: 左花括号
    {
// L225: 浮点/整型最小步长不同
        float lo = (it->type == ITEM_FLOAT) ? FINE_MIN_FLOAT : FINE_MIN_INT;
// L226: 步长缩小 10 倍
        step *= FINE_STEP_DIV;
// L227: 不低于最小步长
        if (step < lo) step = lo;
// L228: if fine 结束
    }
// L229: 长按重复则步长再 ×10
    step *= (repeat ? KEY_STEP_MULT : 1.0f);
// L230: 按方向加减
    s_edit_val += (dir > 0) ? step : -step;
// L231: 钳位 min
    if (s_edit_val < it->min) s_edit_val = it->min;
// L232: 钳位 max
    if (s_edit_val > it->max) s_edit_val = it->max;
// L233: 仅重绘数值列
    draw_value(it, (uint8_t)((s_cursor - s_top) + 1), s_edit_val, MENU_STYLE_EDIT);
// L234: 函数结束
}
// L235: 空行

// L236: 结束编辑：commit 写回或放弃
static void edit_end(bool commit)
// L237: 左花括号
{
// L238: ENTER 确认时一次写回绑定变量
    if (commit && s_edit_item) item_set(s_edit_item, s_edit_val);   // 单次写回
// L239: 回到列表态
    s_nav = NAV_LIST;
// L240: 恢复 TUNING 标题
    draw_title("TUNING");
// L241: 重绘当前行（非编辑）
    draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true, false);
// L242: 函数结束
}
// L243: 空行

// L244: 内部：所有非 ACTION 项设为 def
static void apply_defaults(void)
// L245: 左花括号
{
// L246: 遍历索引
    uint16_t i;
// L247: 循环每项
    for (i = 0; i < menu_item_count; i++)
// L248: 跳过 ACTION
        if (menu_items[i].type != ITEM_ACTION) item_set(&menu_items[i], menu_items[i].def);
// L249: 函数结束
}
// L250: 空行

// L251: Restore Def 菜单动作：应用默认并重绘数值
void menu_action_defaults(void)
// L252: 左花括号
{
// L253: 写回所有 def
    apply_defaults();
// L254: 标题提示 Defaults
    draw_title("Defaults");
// L255: 刷新可见数值
    redraw_visible_values();
// L256: 函数结束
}
// L257: 空行

// L258: 应用单档 Race Preset：先默认再覆盖关键参数
static void apply_preset(const preset_t *p)
// L259: 左花括号
{
// L260: 先恢复菜单 def（High 档与之一致）
    apply_defaults();
// L261: 覆盖 Kp
    steer_kp           = p->kp;
// L262: 覆盖 Kd
    steer_kd           = p->kd;
// L263: D 滤波
    steer_d_filt_alpha = p->d_alpha;
// L264: 图像阈值
    image_threshold    = p->threshold;
// L265: 十字补线
    image_cross_fill   = p->cross_fill;
// L266: Look Far
    steer_look_far     = p->look_far;
// L267: 直行 duty
    drive_duty_base    = p->duty;
// L268: Armed 超时
    drive_stop_time_s  = p->stop_time;
// L269: 函数结束
}
// L270: 空行

// L271: 绘制 NAV_PRESET 子页面
static void draw_preset_page(void)
// L272: 左花括号
{
// L273: 行缓冲
    char row[MENU_COLS + 1];
// L274: 循环变量
    uint8_t i, j, k;
// L275: 空行

// L276: 清屏
    menu_port_clear();
// L277: 标题 PRESET
    draw_title("PRESET");
// L278: 绘制三档名称行
    for (i = 0; i < PRESET_COUNT; i++)
// L279: 左花括号
    {
// L280: 档名
        const char *nm = s_presets[i].name;
// L281: 行填空格
        for (j = 0; j < MENU_COLS; j++) row[j] = ' ';
// L282: C 结尾
        row[MENU_COLS] = '\0';
// L283: 文字从列 2 开始
        j = 2;
// L284: 当前档加 "> "
        if (i == s_preset_cursor) { row[0] = '>'; row[1] = ' '; }
// L285: 拷贝名称
        for (k = 0; nm[k] && j < MENU_COLS; k++) row[j++] = nm[k];
// L286: 绘在第 i+2 行（0 标题，1 空，2 起为档）
        menu_port_draw_text(0, (uint8_t)(i + 2), row,
// L287: 选中样式或普通（续上行）
                            (i == s_preset_cursor) ? MENU_STYLE_SELECTED
// L288: 普通样式（续上行）
                                                   : MENU_STYLE_NORMAL);
// L289: for 结束
    }
// L290: 底部操作提示
    menu_port_draw_text(0, (uint8_t)(PRESET_COUNT + 3),
// L291: 提示文字（续上行）
                        "ENTER:apply  BACK:exit", MENU_STYLE_NORMAL);
// L292: 函数结束
}
// L293: 空行

// L294: Race Preset 入口：切 NAV_PRESET
void menu_action_race_preset(void)
// L295: 左花括号
{
// L296: 光标复位到 Low
    s_preset_cursor = 0;
// L297: 导航态切预设页
    s_nav = NAV_PRESET;
// L298: 绘制预设 UI
    draw_preset_page();
// L299: 函数结束
}
// L300: 空行

// L301: 进入摄像头预览：关其它测试态
void menu_action_camera(void)
// L302: 左花括号
{
// L303: 关对齐测试
    s_align_test_mode = 0;
// L304: 关电机测试
    s_motor_test_mode = 0;
// L305: 关左轮测试
    s_left_test_mode  = 0;
// L306: 开摄像头视图（主循环据此显示图像）
    s_camera_view = 1;
// L307: 函数结束
}
// L308: 空行

// L309: 对齐测试：摄像头 + 停车
void menu_action_align_test(void)
// L310: 左花括号
{
// L311: 关电机测试
    s_motor_test_mode = 0;
// L312: 关左轮测试
    s_left_test_mode  = 0;
// L313: 开对齐模式
    s_align_test_mode = 1;
// L314: 开摄像头
    s_camera_view = 1;
// L315: 停电机
    motor_stop();
// L316: 函数结束
}
// L317: 空行

// L318: 双电机测试模式
void menu_action_motor_test(void)
// L319: 左花括号
{
// L320: 关对齐
    s_align_test_mode = 0;
// L321: 关左轮
    s_left_test_mode  = 0;
// L322: 关摄像头菜单页
    s_camera_view = 0;
// L323: 开电机测试（主循环固定 20% duty）
    s_motor_test_mode = 1;
// L324: 先停车再进入测试
    motor_stop();
// L325: 清屏
    menu_port_clear();
// L326: 标题
    draw_title("Motor Test");
// L327: 说明 duty
    menu_port_draw_text(0, 2, "Duty: 20%", MENU_STYLE_NORMAL);
// L328: BACK 提示
    menu_port_draw_text(0, 4, "BACK: stop", MENU_STYLE_NORMAL);
// L329: 函数结束
}
// L330: 空行

// L331: 注释：仅 pins.h LEFT 两路 PWM，用于辨认物理左右
// 只开 pins.h 里 LEFT 的两路 PWM。转的是哪侧轮 = 软件 LEFT 对应哪侧
// L332: 左轮 PWM 辨识测试
void menu_action_left_test(void)
// L333: 左花括号
{
// L334: 关对齐
    s_align_test_mode = 0;
// L335: 关电机测试
    s_motor_test_mode = 0;
// L336: 关摄像头
    s_camera_view = 0;
// L337: 开左轮测试
    s_left_test_mode = 1;
// L338: 停车
    motor_stop();
// L339: 清屏
    menu_port_clear();
// L340: 标题
    draw_title("Left Test");
// L341: 说明仅 LEFT PWM
    menu_port_draw_text(0, 2, "Only LEFT PWM", MENU_STYLE_NORMAL);
// L342: duty 20%
    menu_port_draw_text(0, 3, "Duty: 20%", MENU_STYLE_NORMAL);
// L343: 提示观察
    menu_port_draw_text(0, 5, "See which side", MENU_STYLE_NORMAL);
// L344: 续行提示
    menu_port_draw_text(0, 6, "spins", MENU_STYLE_NORMAL);
// L345: BACK 退出
    menu_port_draw_text(0, 8, "BACK: stop", MENU_STYLE_NORMAL);
// L346: 函数结束
}
// L347: 空行

// L348: Reset 动作：控制与电机复位
void menu_action_reset(void)
// L349: 左花括号
{
// L350: 控制状态机/init
    control_init();
// L351: 电机复位
    motor_reset();
// L352: 标题反馈
    draw_title("Reset");
// L353: 函数结束
}
// L354: 空行

// L355: 查询摄像头视图标志
uint8_t menu_camera_view(void)
// L356: 左花括号
{
// L357: 返回 s_camera_view
    return s_camera_view;
// L358: 函数结束
}
// L359: 空行

// L360: 查询对齐测试
uint8_t menu_align_test_mode(void)
// L361: 左花括号
{
// L362: 返回标志
    return s_align_test_mode;
// L363: 函数结束
}
// L364: 空行

// L365: 查询电机测试
uint8_t menu_motor_test_mode(void)
// L366: 左花括号
{
// L367: 返回标志
    return s_motor_test_mode;
// L368: 函数结束
}
// L369: 空行

// L370: 查询左轮测试
uint8_t menu_left_test_mode(void)
// L371: 左花括号
{
// L372: 返回标志
    return s_left_test_mode;
// L373: 函数结束
}
// L374: 空行

// L375: 菜单初始化
void menu_init(void)
// L376: 左花括号
{
// L377: 屏、键端口 init
    menu_port_init();
// L378: 所有数值项设为 def
    apply_defaults();
// L379: 上电默认未武装
    drive_armed = 0;
// L380: 清除超时标志
    drive_timed_out = 0;
// L381: FSM 列表态
    s_nav = NAV_LIST;
// L382: 光标在首项
    s_cursor = 0;
// L383: 窗口顶 0
    s_top = 0;
// L384: 绘制完整列表
    draw_list_full();
// L385: 函数结束
}
// L386: 空行

// L387: 单键事件分发：测试模式优先于 FSM
static void menu_handle_key(const menu_key_event_t *ev)
// L388: 左花括号
{
// L389: 电机测试模式：只响应 BACK
    if (s_motor_test_mode)
// L390: 左花括号
    {
// L391: BACK 退出
        if (ev->key == MENU_KEY_BACK)
// L392: 左花括号
        {
// L393: 关模式
            s_motor_test_mode = 0;
// L394: 停车
            motor_stop();
// L395: 恢复列表 UI
            draw_list_full();
// L396: if BACK 结束
        }
// L397: 吞掉其它键
        return;
// L398: if motor 结束
    }
// L399: 空行

// L400: 左轮测试：同样仅 BACK
    if (s_left_test_mode)
// L401: 左花括号
    {
// L402: BACK
        if (ev->key == MENU_KEY_BACK)
// L403: 左花括号
        {
// L404: 关模式
            s_left_test_mode = 0;
// L405: 停车
            motor_stop();
// L406: 恢复列表
            draw_list_full();
// L407: if 结束
        }
// L408: return
        return;
// L409: if left 结束
    }
// L410: 空行

// L411: 摄像头视图：BACK 退出预览
    if (s_camera_view)
// L412: 左花括号
    {
// L413: BACK
        if (ev->key == MENU_KEY_BACK)
// L414: 左花括号
        {
// L415: 关摄像头
            s_camera_view = 0;
// L416: 关对齐（若从 Align Test 进入）
            s_align_test_mode = 0;
// L417: 恢复列表
            draw_list_full();
// L418: if 结束
        }
// L419: return
        return;
// L420: if camera 结束
    }
// L421: 空行

// L422: NAV_PRESET 子页按键
    if (s_nav == NAV_PRESET)
// L423: 左花括号
    {
// L424: UP 上移光标
        if (ev->key == MENU_KEY_UP)
// L425: 左花括号
        {
// L426: 非顶档则减并重绘
            if (s_preset_cursor > 0) { s_preset_cursor--; draw_preset_page(); }
// L427: if UP 结束
        }
// L428: DOWN 下移
        else if (ev->key == MENU_KEY_DOWN)
// L429: 左花括号
        {
// L430: 非底档则加并重绘
            if (s_preset_cursor + 1 < PRESET_COUNT) { s_preset_cursor++; draw_preset_page(); }
// L431: if DOWN 结束
        }
// L432: ENTER 应用当前档
        else if (ev->key == MENU_KEY_ENTER)
// L433: 左花括号
        {
// L434: 写参数
            apply_preset(&s_presets[s_preset_cursor]);
// L435: 回列表态
            s_nav = NAV_LIST;
// L436: 重绘列表
            draw_list_full();
// L437: 标题改为档名反馈
            draw_title(s_presets[s_preset_cursor].name);
// L438: if ENTER 结束
        }
// L439: BACK 放弃
        else if (ev->key == MENU_KEY_BACK)
// L440: 左花括号
        {
// L441: 回列表
            s_nav = NAV_LIST;
// L442: 重绘
            draw_list_full();
// L443: if BACK 结束
        }
// L444: 预设页处理完毕
        return;
// L445: if PRESET 结束
    }
// L446: 空行

// L447: NAV_LIST 主列表
    if (s_nav == NAV_LIST)
// L448: 左花括号
    {
// L449: UP 移动
        if      (ev->key == MENU_KEY_UP)    list_move(-1);
// L450: DOWN
        else if (ev->key == MENU_KEY_DOWN)  list_move(+1);
// L451: ENTER 进入项
        else if (ev->key == MENU_KEY_ENTER) item_enter();
// L452: if LIST 结束
    }
// L453: 否则 NAV_EDIT
    else /* NAV_EDIT */
// L454: 左花括号
    {
// L455: UP 增加值（repeat 放大步长）
        if      (ev->key == MENU_KEY_UP)    edit_adjust(+1, ev->is_repeat);
// L456: DOWN 减少
        else if (ev->key == MENU_KEY_DOWN)  edit_adjust(-1, ev->is_repeat);
// L457: ENTER 确认写回
        else if (ev->key == MENU_KEY_ENTER) edit_end(true);
// L458: BACK 取消
        else if (ev->key == MENU_KEY_BACK)  edit_end(false);
// L459: else EDIT 结束
    }
// L460: 函数结束
}
// L461: 空行

// L462: 主循环任务：扫描键 + 排空事件队列
void menu_task(void)
// L463: 左花括号
{
// L464: 单事件缓冲
    menu_key_event_t ev;
// L465: 空行

// L466: 推进 GPIO 键 FSM（5ms 周期）
    menu_port_key_scan();
// L467: 循环取尽 pending/repeat 事件
    while (1)
// L468: 左花括号
    {
// L469: 取一个事件
        menu_port_scan_keys(&ev);
// L470: 无事件则退出 while
        if (ev.key == MENU_KEY_NONE)
// L471: 左花括号
        {
// L472: break
            break;
// L473: if 结束
        }
// L474: 分发按键
        menu_handle_key(&ev);
// L475: while 继续直到队列空
    }
// L476: 函数结束
}
```
