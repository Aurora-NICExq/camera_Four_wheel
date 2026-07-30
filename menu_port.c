#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "menu_port.h"


#define KEY_COUNT            (4)
#define KEY_ACTIVE_LEVEL     (GPIO_LOW)

typedef struct
{
    gpio_pin_enum pin;
    menu_key_e    map;
    uint8_t       allow_repeat;
    uint8_t       pressed;
    uint8_t       debounce_cnt;
    uint32_t      press_ms;
} key_fsm_t;


static key_fsm_t s_keys[KEY_COUNT] =
{
    { PIN_KEY_UP,    MENU_KEY_UP,    1, 0, 0, 0 },
    { PIN_KEY_DOWN,  MENU_KEY_DOWN,  1, 0, 0, 0 },
    { PIN_KEY_ENTER, MENU_KEY_ENTER, 0, 0, 0, 0 },
    { PIN_KEY_BACK,  MENU_KEY_BACK,  0, 0, 0, 0 },
};

static uint8_t  s_pending = 0;
static uint32_t s_last_scan_ms = 0;
static uint32_t s_last_repeat_ms = 0;
static menu_key_e s_last_key = MENU_KEY_NONE;
static uint8_t    s_last_repeat = 0;

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
            s_keys[i].debounce_cnt = 0;
        }
        else
        {
            s_keys[i].debounce_cnt++;
            if (s_keys[i].debounce_cnt >= KEY_DEBOUNCE_COUNT)
            {
                s_keys[i].pressed = raw;
                s_keys[i].debounce_cnt = 0;

                if (raw)
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


void menu_port_scan_keys(menu_key_event_t *ev)
{
    uint8_t i;

    ev->key = MENU_KEY_NONE;
    ev->is_repeat = 0;


    for (i = 0; i < KEY_COUNT; i++)
    {
        uint8_t mask = (uint8_t)(1u << i);
        if (s_pending & mask)
        {
            s_pending &= (uint8_t)~mask;
            if (s_keys[i].map == MENU_KEY_NONE)
            {
                continue;
            }
            ev->key = s_keys[i].map;
            s_last_key = ev->key;
            s_last_repeat = 0;
            return;
        }
    }


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
                s_last_key = ev->key;
                s_last_repeat = 1;
            }
            break;
        }
    }
}

static const char *key_name(menu_key_e key)
{
    switch (key)
    {
    case MENU_KEY_UP:    return "UP ";
    case MENU_KEY_DOWN:  return "DN ";
    case MENU_KEY_ENTER: return "ENT";
    case MENU_KEY_BACK:  return "BAK";
    default:             return "---";
    }
}

uint8_t menu_port_key_pressed(menu_key_e key)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++)
    {
        if (s_keys[i].map == key)
        {
            return s_keys[i].pressed;
        }
    }
    return 0;
}

void menu_port_draw_key_status(void)
{
    char line[MENU_COLS + 1];
    uint8_t i;
    uint8_t n = 0;

    for (i = 0; i < MENU_COLS; i++)
    {
        line[i] = ' ';
    }
    line[MENU_COLS] = '\0';

    line[n++] = 'K';
    line[n++] = ':';
    for (i = 0; i < KEY_COUNT; i++)
    {
        line[n++] = (s_keys[i].pressed != 0) ? ('1' + i) : '-';
    }
    line[n++] = ' ';
    line[n++] = 'L';
    line[n++] = '=';
    line[n++] = key_name(s_last_key)[0];
    line[n++] = key_name(s_last_key)[1];
    line[n++] = key_name(s_last_key)[2];
    if (s_last_repeat != 0)
    {
        line[n++] = 'R';
    }

    menu_port_draw_text(0, (uint8_t)(MENU_ROWS - 1), line, MENU_STYLE_NORMAL);
}
