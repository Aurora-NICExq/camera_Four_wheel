# menu_port.c 逐行注释

> 行号与源文件一致。

菜单 GPIO 按键端口与 IPS200 显示实现。每键独立消抖 FSM；UP/DOWN 支持长按自动重复。`menu_port_scan_keys` 将物理键映射为 `MENU_KEY_UP/DOWN/ENTER/BACK`。

## 按键 FSM 概要

```
raw GPIO → debounce_cnt 累加 → 达 KEY_DEBOUNCE_COUNT 确认边沿
         → 按下沿：置 pending 位 + 记录 press_ms
         → 长按：allow_repeat 键在 KEY_LONG_PRESS_MS 后以 KEY_REPEAT_MS 周期发 is_repeat=1
```

```c
// L1: 文件头注释
/* menu_port.c */
// L2: 逐飞通用头（GPIO、IPS200、system_getval_us）
#include "zf_common_headfile.h"
// L3: 板级引脚宏 PIN_KEY_*、PIN_IPS200_BL
#include "pins.h"
// L4: KEY_SCAN_PERIOD_MS、KEY_DEBOUNCE_COUNT 等
#include "config.h"
// L5: 端口 API 声明
#include "menu_port.h"
// L6: 空行

// L7: 四键固定数量
#define KEY_COUNT            (4)
// L8: 有效电平为低（上拉输入，按下接地）
#define KEY_ACTIVE_LEVEL     (GPIO_LOW)
// L9: 空行

// L10: 单键状态机结构
typedef struct
// L11: 左花括号
{
// L12: 硬件 GPIO 枚举
    gpio_pin_enum pin;
// L13: 映射到的逻辑菜单键
    menu_key_e    map;
// L14: 1=允许长按重复（仅 UP/DOWN）
    uint8_t       allow_repeat;
// L15: 消抖后稳定态：1=按下 0=释放
    uint8_t       pressed;
// L16: 与 raw 不一致时的计数器
    uint8_t       debounce_cnt;
// L17: 本次按下起始毫秒（长按计时）
    uint32_t      press_ms;
// L18: 结构体结束
} key_fsm_t;
// L19: 空行

// L20: 四键静态表；顺序与 pending 位索引一致
static key_fsm_t s_keys[KEY_COUNT] =
// L21: 初始化列表开始
{
// L22: KEY1 上：P13_3，允许长按重复
    { PIN_KEY_UP,    MENU_KEY_UP,    1, 0, 0, 0 },   /* KEY1 P13_3 */
// L23: KEY2 下：P11_9，允许重复
    { PIN_KEY_DOWN,  MENU_KEY_DOWN,  1, 0, 0, 0 },   /* KEY2 P11_9 */
// L24: KEY3 确认：P11_10，不重复
    { PIN_KEY_ENTER, MENU_KEY_ENTER, 0, 0, 0, 0 },   /* KEY3 P11_10 */
// L25: KEY4 返回：P11_11，不重复
    { PIN_KEY_BACK,  MENU_KEY_BACK,  0, 0, 0, 0 },   /* KEY4 P11_11 */
// L26: 列表结束
};
// L27: 空行

// L28: 待处理按键位掩码：bit i 表示键 i 有新按下沿
static uint8_t  s_pending = 0;
// L29: 上次 key_scan_once 的毫秒时间
static uint32_t s_last_scan_ms = 0;
// L30: 上次发出 repeat 事件的毫秒时间（全局节流）
static uint32_t s_last_repeat_ms = 0;
// L31: 空行

// L32: 读单引脚是否处于“按下”态
static uint8_t key_pressed(gpio_pin_enum pin)
// L33: 左花括号
{
// L34: 低电平返回 1，否则 0
    return (gpio_get_level(pin) == KEY_ACTIVE_LEVEL) ? 1u : 0u;
// L35: 函数结束
}
// L36: 空行

// L37: 一轮扫描：更新四键消抖状态
static void key_scan_once(void)
// L38: 左花括号
{
// L39: 循环变量
    uint8_t i;
// L40: 空行

// L41: 遍历每个键
    for (i = 0; i < KEY_COUNT; i++)
// L42: 左花括号
    {
// L43: 当前原始电平
        uint8_t raw = key_pressed(s_keys[i].pin);
// L44: 空行

// L45: raw 与已确认 pressed 相同：稳定，清零抖动计数
        if (raw == s_keys[i].pressed)
// L46: 左花括号
        {
// L47: 重置 debounce
            s_keys[i].debounce_cnt = 0;
// L48: if 结束
        }
// L49: 电平变化：进入消抖
        else
// L50: 左花括号
        {
// L51: 累加消抖样本
            s_keys[i].debounce_cnt++;
// L52: 达到 config.h 计算的 KEY_DEBOUNCE_COUNT（20ms/5ms=4）
            if (s_keys[i].debounce_cnt >= KEY_DEBOUNCE_COUNT)
// L53: 左花括号
            {
// L54: 确认新稳定态
                s_keys[i].pressed = raw;
// L55: 消抖计数清零
                s_keys[i].debounce_cnt = 0;
// L56: 空行

// L57: 若为按下沿（稳定为 1）
                if (raw)
// L58: 左花括号
                {
// L59: 置 pending 对应位，等待 scan_keys 消费
                    s_pending |= (uint8_t)(1u << i);
// L60: 记录按下时刻供长按判定
                    s_keys[i].press_ms = menu_port_millis();
// L61: if(raw) 结束
                }
// L62: debounce 达标块结束
            }
// L63: else 消抖分支结束
        }
// L64: for 单次迭代结束
    }
// L65: 函数结束
}
// L66: 空行

// L67: 按键 GPIO 初始化（仅首次 init 调用）
static void menu_port_keys_init(void)
// L68: 左花括号
{
// L69: 循环变量
    uint8_t i;
// L70: 空行

// L71: 配置四键为上拉输入，默认高电平
    for (i = 0; i < KEY_COUNT; i++)
// L72: 左花括号
    {
// L73: GPI 输入 + 内部上拉
        gpio_init(s_keys[i].pin, GPI, GPIO_HIGH, GPI_PULL_UP);
// L74: FSM 初始为未按下
        s_keys[i].pressed = 0;
// L75: 消抖计数清零
        s_keys[i].debounce_cnt = 0;
// L76: 按下时间清零
        s_keys[i].press_ms = 0;
// L77: for 结束
    }
// L78: 清空待处理队列
    s_pending = 0;
// L79: 初始化扫描时间戳
    s_last_scan_ms = menu_port_millis();
// L80: 初始化重复时间戳
    s_last_repeat_ms = 0;
// L81: 函数结束
}
// L82: 空行

// L83: 毫秒时钟：微秒计时除以 1000
uint32_t menu_port_millis(void)
// L84: 左花括号
{
// L85: 使用 system_getval_us() 挂钟，符合工程 hal 约束
    return (uint32_t)(system_getval_us() / 1000u);
// L86: 函数结束
}
// L87: 空行

// L88: 端口总初始化：屏 + 背光 + 键（键只 init 一次）
void menu_port_init(void)
// L89: 左花括号
{
// L90: 静态标志，防止重复 gpio_init 键
    static uint8_t keys_ready = 0;
// L91: 空行

// L92: IPS200 设为竖屏方向
    ips200_set_dir(IPS200_PORTAIT);
// L93: 按 pins.h 的 SPI 类型初始化屏
    ips200_init(IPS200_CONNECT_TYPE);
// L94: 本板背光脚 P20_14 推挽输出拉高
    gpio_init(PIN_IPS200_BL, GPO, GPIO_HIGH, GPO_PUSH_PULL);
// L95: 8x16 字体
    ips200_set_font(IPS200_8X16_FONT);
// L96: 清屏
    ips200_clear();
// L97: 空行

// L98: 首次才初始化按键
    if (!keys_ready)
// L99: 左花括号
    {
// L100: 调用键 FSM 初始化
        menu_port_keys_init();
// L101: 标记已完成
        keys_ready = 1;
// L102: if 结束
    }
// L103: 函数结束
}
// L104: 空行

// L105: 主循环周期性调用：按 KEY_SCAN_PERIOD_MS（5ms）节流派发 scan
void menu_port_key_scan(void)
// L106: 左花括号
{
// L107: 当前毫秒
    uint32_t now_ms = menu_port_millis();
// L108: 空行

// L109: 距上次扫描已满一个周期
    if ((now_ms - s_last_scan_ms) >= (uint32)KEY_SCAN_PERIOD_MS)
// L110: 左花括号
    {
// L111: 更新上次扫描时间
        s_last_scan_ms = now_ms;
// L112: 执行一轮四键消抖
        key_scan_once();
// L113: if 结束
    }
// L114: 函数结束
}
// L115: 空行

// L116: 清屏封装
void menu_port_clear(void)
// L117: 左花括号
{
// L118: 调用 IPS200 驱动清屏
    ips200_clear();
// L119: 函数结束
}
// L120: 空行

// L121: 字符网格坐标转像素绘制字符串；style 暂未用于反色
void menu_port_draw_text(uint8_t col, uint8_t row, const char *s, menu_style_e style)
// L122: 左花括号
{
// L123: 忽略样式（未来可接前景/背景色）
    (void)style;
// L124: col*8、row*16 为像素原点
    ips200_show_string((uint16)(col * 8), (uint16)(row * 16), s);
// L125: 函数结束
}
// L126: 空行

// L127: 绘制有符号整数
void menu_port_draw_int(uint8_t col, uint8_t row, int32_t v, uint8_t width, menu_style_e style)
// L128: 左花括号
{
// L129: 显示位数：width-1 或至少 1
    uint8_t num = (width > 1) ? (uint8_t)(width - 1) : 1;
// L130: 上限 10 位防溢出
    if (num > 10) num = 10;
// L131: 忽略样式
    (void)style;
// L132: 调用逐飞 show_int
    ips200_show_int((uint16)(col * 8), (uint16)(row * 16), v, num);
// L133: 函数结束
}
// L134: 空行

// L135: 绘制无符号整数
void menu_port_draw_uint(uint8_t col, uint8_t row, uint32_t v, uint8_t width, menu_style_e style)
// L136: 左花括号
{
// L137: 位数限制在 1~10
    uint8_t num = (width < 1) ? 1 : (width > 10 ? 10 : (uint8_t)width);
// L138: 忽略样式
    (void)style;
// L139: 调用 show_uint
    ips200_show_uint((uint16)(col * 8), (uint16)(row * 16), v, num);
// L140: 函数结束
}
// L141: 空行

// L142: 绘制浮点数
void menu_port_draw_float(uint8_t col, uint8_t row, float v, uint8_t int_w, uint8_t dec_w, menu_style_e style)
// L143: 左花括号
{
// L144: 忽略样式
    (void)style;
// L145: 转 double 传给库函数
    ips200_show_float((uint16)(col * 8), (uint16)(row * 16), (double)v, (uint8)int_w, (uint8)dec_w);
// L146: 函数结束
}
// L147: 空行

// L148: 注释：物理键到逻辑键映射说明
// 按键 -> 菜单事件：KEY1=上 KEY2=下 KEY3=确认 KEY4=返回
// L149: 从 pending 或长按重复中取一个事件填入 ev
void menu_port_scan_keys(menu_key_event_t *ev)
// L150: 左花括号
{
// L151: 循环变量
    uint8_t i;
// L152: 空行

// L153: 默认无键
    ev->key = MENU_KEY_NONE;
// L154: 默认非重复
    ev->is_repeat = 0;
// L155: 空行

// L156: 第一遍：优先消费 pending 中的按下沿（一次一个）
    for (i = 0; i < KEY_COUNT; i++)
// L157: 左花括号
    {
// L158: 第 i 键的 pending 掩码
        uint8_t mask = (uint8_t)(1u << i);
// L159: 若有待处理按下沿
        if (s_pending & mask)
// L160: 左花括号
        {
// L161: 清除该位
            s_pending &= (uint8_t)~mask;
// L162: 若映射为 NONE 则跳过（当前表无此种情况）
            if (s_keys[i].map == MENU_KEY_NONE)
// L163: 左花括号
            {
// L164: continue 找下一 pending
                continue;
// L165: if 结束
            }
// L166: 输出逻辑键
            ev->key = s_keys[i].map;
// L167: 按下沿事件，立即返回
            return;
// L168: if pending 结束
        }
// L169: for 第一遍结束
    }
// L170: 空行

// L171: 第二遍：长按自动重复（只处理第一个仍按下的 allow_repeat 键）
    for (i = 0; i < KEY_COUNT; i++)
// L172: 左花括号
    {
// L173: 允许重复且当前仍按住
        if (s_keys[i].allow_repeat && s_keys[i].pressed)
// L174: 左花括号
        {
// L175: 当前时间
            uint32_t now = menu_port_millis();
// L176: 按住超过 KEY_LONG_PRESS_MS 且距上次 repeat 超过 KEY_REPEAT_MS
            if (((now - s_keys[i].press_ms) >= (uint32)KEY_LONG_PRESS_MS) &&
// L177: 重复间隔条件（续上一行）
                ((now - s_last_repeat_ms) >= (uint32)KEY_REPEAT_MS))
// L178: 左花括号
            {
// L179: 更新全局 repeat 时间戳
                s_last_repeat_ms = now;
// L180: 输出相同逻辑键
                ev->key = s_keys[i].map;
// L181: 标记为重复事件，menu.c 中 edit_adjust 会放大步长
                ev->is_repeat = 1;
// L182: if 长按结束
            }
// L183: 只检查第一个符合条件的重复键后 break
            break;
// L184: if allow_repeat 结束
        }
// L185: for 第二遍结束
    }
// L186: 函数结束（无事件则 key 仍为 NONE）
}
```
