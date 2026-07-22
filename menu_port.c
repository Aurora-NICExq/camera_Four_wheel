/* menu_port.c - menu HAL (IPS200 + keys) */
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "menu_port.h"

#define MENU_FLASH_SECTOR   (0)      // 与逐飞 EEPROM demo 一致：sector 0 / page 8（DFLASH 挑一页别人不用的）
#define MENU_FLASH_PAGE     (8)
#define KEY_EVQ_LEN         (8)

typedef enum
{
    KS_IDLE = 0,
    KS_DEBOUNCE,
    KS_HELD,
    KS_REPEAT,
} key_fsm_state_e;

typedef struct
{
    gpio_pin_enum   pin;
    menu_key_e      map;
    uint8_t         allow_repeat;
    key_fsm_state_e state;
    uint32_t        t_state;
} key_fsm_t;

static volatile uint32_t s_tick_ms = 0;
static volatile uint8_t s_evq_head = 0;
static volatile uint8_t s_evq_tail = 0;
static menu_key_event_t s_evq[KEY_EVQ_LEN];

static key_fsm_t s_keys[4] =
{
    { PIN_KEY_UP,    MENU_KEY_UP,    1, KS_IDLE, 0 },
    { PIN_KEY_DOWN,  MENU_KEY_DOWN,  1, KS_IDLE, 0 },
    { PIN_KEY_ENTER, MENU_KEY_ENTER, 0, KS_IDLE, 0 },
    { PIN_KEY_BACK,  MENU_KEY_BACK,  0, KS_IDLE, 0 },
};

static uint8_t key_pressed(gpio_pin_enum pin)
{
    return (gpio_get_level(pin) == GPIO_LOW) ? 1u : 0u;
}

static void key_evq_push(menu_key_e key, uint8_t is_repeat)
{
    uint8_t next = (uint8_t)((s_evq_tail + 1u) % KEY_EVQ_LEN);

    if (next == s_evq_head)
    {
        return;
    }

    s_evq[s_evq_tail].key = key;
    s_evq[s_evq_tail].is_repeat = is_repeat;
    s_evq_tail = next;
}

static void key_fsm_tick_one(key_fsm_t *k)
{
    uint32_t now = s_tick_ms;
    uint8_t raw = key_pressed(k->pin);

    switch (k->state)
    {
    case KS_IDLE:
        if (raw)
        {
            k->state = KS_DEBOUNCE;
            k->t_state = now;
        }
        break;

    case KS_DEBOUNCE:
        if (!raw)
        {
            k->state = KS_IDLE;
        }
        else if ((now - k->t_state) >= (uint32)KEY_DEBOUNCE_MS)
        {
            k->state = KS_HELD;
            k->t_state = now;
            key_evq_push(k->map, 0);
        }
        break;

    case KS_HELD:
        if (!raw)
        {
            k->state = KS_IDLE;
        }
        else if (k->allow_repeat && (now - k->t_state) >= (uint32)KEY_LONG_MS)
        {
            k->state = KS_REPEAT;
            k->t_state = now;
            key_evq_push(k->map, 1);
        }
        break;

    case KS_REPEAT:
        if (!raw)
        {
            k->state = KS_IDLE;
        }
        else if ((now - k->t_state) >= (uint32)KEY_REPEAT_MS)
        {
            k->t_state = now;
            key_evq_push(k->map, 1);
        }
        break;

    default:
        k->state = KS_IDLE;
        break;
    }
}

static void menu_port_keys_init(void)
{
    uint8_t i;

    for (i = 0; i < 4u; i++)
    {
        gpio_init(s_keys[i].pin, GPI, GPIO_HIGH, GPI_PULL_UP);
        s_keys[i].state = KS_IDLE;
        s_keys[i].t_state = 0;
    }

    s_evq_head = 0;
    s_evq_tail = 0;
    s_tick_ms = 0;
    pit_ms_init(CCU60_CH0, KEY_SCAN_MS);
}

void menu_port_key_tick(void)
{
    uint8_t i;

    s_tick_ms += (uint32)KEY_SCAN_MS;
    for (i = 0; i < 4u; i++)
    {
        key_fsm_tick_one(&s_keys[i]);
    }
}

uint32_t menu_port_millis(void)
{
    return (uint32_t)s_tick_ms;
}

void menu_port_init(void)
{
    static uint8_t keys_ready = 0;

    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_CONNECT_TYPE);
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
    /* 扫描在 CCU60_CH0 PIT ISR 中完成；此处保留 API 兼容 */
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

// Input
//   KEY_1=UP  KEY_2=DOWN  KEY_3=ENTER  KEY_4=BACK
//   5ms PIT 状态机：消抖后立刻触发；UP/DOWN 长按连发（is_repeat=1 -> 10x 步进）。
void menu_port_scan_keys(menu_key_event_t *ev)
{
    ev->key = MENU_KEY_NONE;
    ev->is_repeat = 0;

    if (s_evq_head == s_evq_tail)
    {
        return;
    }

    *ev = s_evq[s_evq_head];
    s_evq_head = (uint8_t)((s_evq_head + 1u) % KEY_EVQ_LEN);
}

uint8_t menu_port_flash_write(const uint32_t *buf, uint16_t count)
{
    uint16_t i;
    if (count > EEPROM_PAGE_LENGTH) return 0;

    flash_buffer_clear();
    for (i = 0; i < count; i++)
        flash_union_buffer[i].uint32_type = (uint32)buf[i];

    flash_erase_page(MENU_FLASH_SECTOR, MENU_FLASH_PAGE);
    return (uint8_t)flash_write_page_from_buffer(MENU_FLASH_SECTOR, MENU_FLASH_PAGE);
}

void menu_port_flash_read(uint32_t *buf, uint16_t count)
{
    uint16_t i;
    if (count > EEPROM_PAGE_LENGTH) count = EEPROM_PAGE_LENGTH;

    flash_read_page_to_buffer(MENU_FLASH_SECTOR, MENU_FLASH_PAGE);
    for (i = 0; i < count; i++)
        buf[i] = (uint32_t)flash_union_buffer[i].uint32_type;
}
