/* menu_port.c - menu HAL (IPS200 + keys) */
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "menu_port.h"

/* ---- 按键输入：四个独立按键（上拉输入，按下为低） --------------------
 * 扫描/消抖/事件模型移植自另一块板（五向摇杆）实测可用的工程：
 *   - 每 KEY_SCAN_PERIOD_MS 采样一次；
 *   - 采样与稳定态连续 KEY_DEBOUNCE_COUNT 次不一致才翻转稳定态；
 *   - 仅在 释放->按下 沿置一个挂起事件位（挂起位保持到被读取，不会过期丢失）；
 *   - 长按不重复产生按下沿；UP/DOWN 连发在读取侧按稳定态+时间生成。 */
#define KEY_COUNT            (4)
#define KEY_ACTIVE_LEVEL     (GPIO_LOW)

typedef struct
{
    gpio_pin_enum pin;
    menu_key_e    map;            /* 映射的菜单事件；MENU_KEY_NONE = 不用 */
    uint8_t       allow_repeat;   /* 1 = 长按连发（仅 UP/DOWN） */
    uint8_t       pressed;        /* 消抖后的稳定按下状态 */
    uint8_t       debounce_cnt;   /* 与稳定态不一致的连续采样计数 */
    uint32_t      press_ms;       /* 稳定态变为按下的时刻（连发计时起点） */
} key_fsm_t;

/* 数组序即挂起位序、事件优先级序 */
static key_fsm_t s_keys[KEY_COUNT] =
{
    { PIN_KEY_UP,    MENU_KEY_UP,    1, 0, 0, 0 },   /* KEY1 P13_3 */
    { PIN_KEY_DOWN,  MENU_KEY_DOWN,  1, 0, 0, 0 },   /* KEY2 P11_9 */
    { PIN_KEY_ENTER, MENU_KEY_ENTER, 0, 0, 0, 0 },   /* KEY3 P11_10 */
    { PIN_KEY_BACK,  MENU_KEY_BACK,  0, 0, 0, 0 },   /* KEY4 P11_11 */
};

static uint8_t  s_pending = 0;         /* bit i = s_keys[i] 的按下沿事件 */
static uint32_t s_last_scan_ms = 0;
static uint32_t s_last_repeat_ms = 0;

static uint8_t key_pressed(gpio_pin_enum pin)
{
    return (gpio_get_level(pin) == KEY_ACTIVE_LEVEL) ? 1u : 0u;
}

static void key_scan_once(void)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++)
    {
        uint8_t raw = key_pressed(s_keys[i].pin);

        if (raw == s_keys[i].pressed)
        {
            s_keys[i].debounce_cnt = 0;        /* 与稳定态一致：取消未完成的翻转 */
        }
        else
        {
            s_keys[i].debounce_cnt++;
            if (s_keys[i].debounce_cnt >= KEY_DEBOUNCE_COUNT)
            {
                s_keys[i].pressed = raw;
                s_keys[i].debounce_cnt = 0;

                if (raw)                       /* 仅 释放->按下 沿生成事件 */
                {
                    s_pending |= (uint8_t)(1u << i);
                    s_keys[i].press_ms = menu_port_millis();
                }
            }
        }
    }
}

static void menu_port_keys_init(void)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++)
    {
        gpio_init(s_keys[i].pin, GPI, GPIO_HIGH, GPI_PULL_UP);
        s_keys[i].pressed = 0;
        s_keys[i].debounce_cnt = 0;
        s_keys[i].press_ms = 0;
    }
    s_pending = 0;
    s_last_scan_ms = menu_port_millis();
    s_last_repeat_ms = 0;
}

uint32_t menu_port_millis(void)
{
    return (uint32_t)(system_getval_us() / 1000u);
}

void menu_port_init(void)
{
    static uint8_t keys_ready = 0;

    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_CONNECT_TYPE);
    gpio_init(PIN_IPS200_BL, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();

    if (!keys_ready)
    {
        menu_port_keys_init();
        keys_ready = 1;
    }
}

void menu_port_key_scan(void)
{
    uint32_t now_ms = menu_port_millis();

    /* 实测工程以 delay(5ms) 定步调用 joystick_scan()；此处用墙钟等效：
     * 每个周期最多扫一次，不补扫——消抖依赖"相邻采样间隔≥扫描周期"。 */
    if ((now_ms - s_last_scan_ms) >= (uint32)KEY_SCAN_PERIOD_MS)
    {
        s_last_scan_ms = now_ms;
        key_scan_once();
    }
}

void menu_port_clear(void)
{
    ips200_clear();
}

void menu_port_draw_text(uint8_t col, uint8_t row, const char *s, menu_style_e style)
{
    (void)style;
    ips200_show_string((uint16)(col * 8), (uint16)(row * 16), s);
}

void menu_port_draw_int(uint8_t col, uint8_t row, int32_t v, uint8_t width, menu_style_e style)
{
    uint8_t num = (width > 1) ? (uint8_t)(width - 1) : 1;
    if (num > 10) num = 10;
    (void)style;
    ips200_show_int((uint16)(col * 8), (uint16)(row * 16), v, num);
}

void menu_port_draw_uint(uint8_t col, uint8_t row, uint32_t v, uint8_t width, menu_style_e style)
{
    uint8_t num = (width < 1) ? 1 : (width > 10 ? 10 : (uint8_t)width);
    (void)style;
    ips200_show_uint((uint16)(col * 8), (uint16)(row * 16), v, num);
}

void menu_port_draw_float(uint8_t col, uint8_t row, float v, uint8_t int_w, uint8_t dec_w, menu_style_e style)
{
    (void)style;
    ips200_show_float((uint16)(col * 8), (uint16)(row * 16), (double)v, (uint8)int_w, (uint8)dec_w);
}

// 按键 -> 菜单事件：KEY1=上 KEY2=下 KEY3=确认 KEY4=返回
void menu_port_scan_keys(menu_key_event_t *ev)
{
    uint8_t i;

    ev->key = MENU_KEY_NONE;
    ev->is_repeat = 0;

    /* 1) 先取按下沿事件：取一个并清其挂起位 */
    for (i = 0; i < KEY_COUNT; i++)
    {
        uint8_t mask = (uint8_t)(1u << i);
        if (s_pending & mask)
        {
            s_pending &= (uint8_t)~mask;
            if (s_keys[i].map == MENU_KEY_NONE)
            {
                continue;              /* 未映射按键：丢弃，继续找下一个 */
            }
            ev->key = s_keys[i].map;
            return;
        }
    }

    /* 2) 无新按下沿：UP/DOWN 稳定按住超过 KEY_LONG_PRESS_MS 后，
     *    每 KEY_REPEAT_MS 生成一个 is_repeat=1 事件（菜单按 10 倍步长调整）。 */
    for (i = 0; i < KEY_COUNT; i++)
    {
        if (s_keys[i].allow_repeat && s_keys[i].pressed)
        {
            uint32_t now = menu_port_millis();
            if (((now - s_keys[i].press_ms) >= (uint32)KEY_LONG_PRESS_MS) &&
                ((now - s_last_repeat_ms) >= (uint32)KEY_REPEAT_MS))
            {
                s_last_repeat_ms = now;
                ev->key = s_keys[i].map;
                ev->is_repeat = 1;
            }
            break;                     /* 只看优先级最高的一个按住键 */
        }
    }
}

