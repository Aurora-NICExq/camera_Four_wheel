/* menu_port.h - menu hardware abstraction */
#ifndef _menu_port_h_
#define _menu_port_h_

#include <stdint.h>
#include <stdbool.h>

//--------------------------------------------------------------------------------------------------------------------
// Screen geometry : 2.0" IPS200 竖屏 240x320, 8x16 font  ->  30 columns x 20 rows.
// Coordinates handed to the draw functions are CHARACTER cells (col 0..29, row 0..19).
//--------------------------------------------------------------------------------------------------------------------
#define MENU_COLS   (30)       // 240 / 8 = 30  (竖屏宽)
#define MENU_ROWS   (20)       // 320 / 16 = 20 (竖屏高)

// Drawing style for a text/number cell run. On a colour IPS200 the port translates
// these into text-level indicators (cursor marker, title decoration, edit prefix).
typedef enum
{
    MENU_STYLE_NORMAL,      // plain text
    MENU_STYLE_SELECTED,    // cursor row: prefix ">"  (inverted background not viable on IPS200)
    MENU_STYLE_EDIT,        // edit-mode row: prefix ">" + title bar shows "[E] name"
    MENU_STYLE_TITLE,       // header bar: drawn as "== NAME ==" line
} menu_style_e;

// A single logical key event produced by the port (debounced, edge/repeat resolved).
typedef enum { MENU_KEY_NONE, MENU_KEY_UP, MENU_KEY_DOWN, MENU_KEY_ENTER, MENU_KEY_BACK } menu_key_e;

typedef struct
{
    menu_key_e key;
    uint8_t    is_repeat;   // 1 == long-press auto-repeat tick (engine applies 10x step)
} menu_key_event_t;

//--------------------------------------------------------------------------------------------------------------------
// Lifecycle / time base
//--------------------------------------------------------------------------------------------------------------------
void     menu_port_init(void);          // display + font
uint32_t menu_port_millis(void);        // free-running millisecond counter

// Call this every frame to scan keys (reuses the existing key_scanner from motor_hw_init).
void     menu_port_key_scan(void);

//--------------------------------------------------------------------------------------------------------------------
// Display primitives -- (col,row) are character cells. Numbers are drawn with the SeekFree ips200 show
// functions (never sprintf). Draw a fixed `width` so a shorter new value cleanly overwrites a longer old one.
//--------------------------------------------------------------------------------------------------------------------
void menu_port_clear(void);
void menu_port_draw_text (uint8_t col, uint8_t row, const char *s, menu_style_e style);
void menu_port_draw_int  (uint8_t col, uint8_t row, int32_t  v, uint8_t width, menu_style_e style);
void menu_port_draw_uint (uint8_t col, uint8_t row, uint32_t v, uint8_t width, menu_style_e style);
void menu_port_draw_float(uint8_t col, uint8_t row, float v, uint8_t int_w, uint8_t dec_w, menu_style_e style);

//--------------------------------------------------------------------------------------------------------------------
// Input : returns at most one event per call (priority ENTER > BACK > UP > DOWN).
//--------------------------------------------------------------------------------------------------------------------
void menu_port_scan_keys(menu_key_event_t *ev);

//--------------------------------------------------------------------------------------------------------------------
// Flash : one fixed page (matches the official SeekFree EEPROM demo). Data is an array of 32-bit words.
//--------------------------------------------------------------------------------------------------------------------
uint8_t menu_port_flash_write(const uint32_t *buf, uint16_t count);   // erase+write; returns 1 on success
void    menu_port_flash_read (uint32_t *buf, uint16_t count);         // read `count` words into buf

#endif
