/* menu_port.c - menu HAL (IPS200 + keys) */
#include "zf_common_headfile.h"
#include "pins.h"
#include "config.h"
#include "menu_port.h"

#define MENU_FLASH_SECTOR    (0)
#define MENU_FLASH_PAGE      (8)
#define KEY_EVQ_LEN          (8)

static volatile uint8_t s_evq_head = 0;
static volatile uint8_t s_evq_tail = 0;
static menu_key_event_t s_evq[KEY_EVQ_LEN];
static uint32_t s_last_scan_ms = 0;
static uint32_t s_last_repeat_ms = 0;

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

static void key_poll_library(void)
{
    key_state_enum up;
    key_state_enum down;

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_3);
        key_evq_push(MENU_KEY_ENTER, 0);
        return;
    }
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_4);
        key_evq_push(MENU_KEY_BACK, 0);
        return;
    }

    up = key_get_state(KEY_1);
    down = key_get_state(KEY_2);

    if (up == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);
        key_evq_push(MENU_KEY_UP, 0);
        return;
    }
    if (down == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        key_evq_push(MENU_KEY_DOWN, 0);
        return;
    }

    if (up == KEY_LONG_PRESS || down == KEY_LONG_PRESS)
    {
        uint32_t now = menu_port_millis();
        if ((now - s_last_repeat_ms) >= (uint32)KEY_REPEAT_MS)
        {
            s_last_repeat_ms = now;
            key_evq_push((up == KEY_LONG_PRESS) ? MENU_KEY_UP : MENU_KEY_DOWN, 1);
        }
    }
}

static void menu_port_keys_init(void)
{
    key_init(KEY_SCAN_PERIOD_MS);
    s_evq_head = 0;
    s_evq_tail = 0;
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
    uint8_t iter = 0;

    while (((now_ms - s_last_scan_ms) >= (uint32)KEY_SCAN_PERIOD_MS) && (iter < 8u))
    {
        s_last_scan_ms += (uint32)KEY_SCAN_PERIOD_MS;
        key_scanner();
        key_poll_library();
        iter++;
        now_ms = menu_port_millis();
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

// KEY_1=UP KEY_2=DOWN KEY_3=ENTER KEY_4=BACK（库内 KEY_LIST = P20_6,P20_7,P11_2,P11_3）
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
