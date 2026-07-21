/* menu_port.c - menu HAL (IPS200 + keys) */
#include "zf_common_headfile.h"
#include "pins.h"
#include "menu_port.h"

#define MENU_FLASH_SECTOR   (0)      // 与逐飞 EEPROM demo 一致：sector 0 / page 8（DFLASH 挑一页别人不用的）
#define MENU_FLASH_PAGE     (8)
#define KEY_REPEAT_MS       (120)    // 长按自动重复周期

uint32_t menu_port_millis(void)     // STM 系统定时器（motor_hw_init 已 system_start）
{
    return (uint32_t)(system_getval_us() / 1000u);
}

void menu_port_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_CONNECT_TYPE);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();
}

void menu_port_key_scan(void)
{
    key_scanner();
}

// Display -- 2-inch colour IPS200.  Character coordinates (col, row) are converted to pixel coordinates.
//   col → x = col * 8      (8x16 font, 8 pixels per character wide)
//   row → y = row * 16     (16 pixels per character tall)
//
//  Style mapping (colour LCD -- no monochrome inverted background):
//    NORMAL   : plain text
//    SELECTED : plain text, but the engine's label already includes ">" at col 0 for cursor indication
//    EDIT     : same as SELECTED; the engine draws the title bar with "[E]" prefix
//    TITLE    : same as NORMAL; the engine decorates the title (e.g. "== NAME ==")
//
//  The engine's build_label() and draw_title*() functions handle all visual distinction. The port
//  layer simply draws the text as given at the requested position.

void menu_port_clear(void)
{
    ips200_clear();
}

void menu_port_draw_text(uint8_t col, uint8_t row, const char *s, menu_style_e style)
{
    (void)style;   // colour: style differences are handled at the engine level (label markup)
    ips200_show_string((uint16)(col * 8), (uint16)(row * 16), s);
}

void menu_port_draw_int(uint8_t col, uint8_t row, int32_t v, uint8_t width, menu_style_e style)
{
    // ips200_show_int draws num + 1 columns (extra slot for sign), so to fill exactly
    // `width` columns pass num = width - 1.
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
//   ENTER/BACK act on a short press only. UP/DOWN: short press = one 1x event; a held key produces throttled
//   repeat events (is_repeat=1 -> engine applies 10x step). The library latches KEY_LONG_PRESS while held.
void menu_port_scan_keys(menu_key_event_t *ev)
{
    static uint32 last_repeat_ms = 0;
    key_state_enum up, down;

    ev->key = MENU_KEY_NONE;
    ev->is_repeat = 0;

    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) { key_clear_state(KEY_3); ev->key = MENU_KEY_ENTER; return; }
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) { key_clear_state(KEY_4); ev->key = MENU_KEY_BACK;  return; }

    up   = key_get_state(KEY_1);
    down = key_get_state(KEY_2);

    if (up   == KEY_SHORT_PRESS) { key_clear_state(KEY_1); ev->key = MENU_KEY_UP;   return; }
    if (down == KEY_SHORT_PRESS) { key_clear_state(KEY_2); ev->key = MENU_KEY_DOWN; return; }

    if (up == KEY_LONG_PRESS || down == KEY_LONG_PRESS)
    {
        uint32 now = menu_port_millis();
        if ((uint32)(now - last_repeat_ms) >= KEY_REPEAT_MS)     // throttle held-key repeats
        {
            last_repeat_ms = now;
            ev->key = (up == KEY_LONG_PRESS) ? MENU_KEY_UP : MENU_KEY_DOWN;
            ev->is_repeat = 1;
        }
        // do NOT clear the long-press state: it auto-latches while held; releasing resets it in the driver.
    }
}

// Flash  (fixed DFLASH page ; data is an array of 32-bit words, stored via the union buffer)
uint8_t menu_port_flash_write(const uint32_t *buf, uint16_t count)
{
    uint16_t i;
    if (count > EEPROM_PAGE_LENGTH) return 0;

    flash_buffer_clear();
    for (i = 0; i < count; i++)
        flash_union_buffer[i].uint32_type = (uint32)buf[i];

    flash_erase_page(MENU_FLASH_SECTOR, MENU_FLASH_PAGE);           // erase before write (required)
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
