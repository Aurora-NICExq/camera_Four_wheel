/*********************************************************************************************************************
 * menu_port.h  --  Hardware abstraction interface for the menu engine.
 *
 * The engine (menu.c) talks to hardware ONLY through the functions declared here. This header is
 * hardware independent (pure <stdint.h>). The single implementation menu_port.c is the ONLY file that
 * includes SeekFree headers -- so porting to another MCU/library means editing menu_port.c and nothing else.
 ********************************************************************************************************************/
#ifndef _menu_port_h_
#define _menu_port_h_

#include <stdint.h>
#include <stdbool.h>

//--------------------------------------------------------------------------------------------------------------------
// Screen geometry : 2.0" display, 240x320, 6x8 font  ->  40 columns x 40 rows.
// Coordinates handed to the draw functions are CHARACTER cells (col 0..39, row 0..39).
//--------------------------------------------------------------------------------------------------------------------
#define MENU_COLS   (40)       // 240 / 6 = 40  (with 6x8 font)
#define MENU_ROWS   (40)       // 320 / 8 = 40

// Drawing style for a text/number cell run. On a monochrome OLED the port translates
// these into text-level indicators (cursor marker, title decoration, edit prefix).
typedef enum
{
    MENU_STYLE_NORMAL,      // plain text
    MENU_STYLE_SELECTED,    // cursor row: prefix ">"  (inverted background not viable on OLED)
    MENU_STYLE_EDIT,        // edit-mode row: prefix ">" + title bar shows "[EDIT] name"
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
void     menu_port_init(void);          // display + keys + PIT time base
uint32_t menu_port_millis(void);        // free-running millisecond counter (advanced by the 10 ms PIT)

// Call this from the 10 ms PIT ISR (cc60_pit_ch0_isr in isr.c). It runs key_scanner() and advances the tick.
void     menu_port_pit_10ms_isr(void);

//--------------------------------------------------------------------------------------------------------------------
// Display primitives -- (col,row) are character cells. Numbers are drawn with the SeekFree oled show
// functions (never sprintf). Draw a fixed `width` so a shorter new value cleanly overwrites a longer old one.
//--------------------------------------------------------------------------------------------------------------------
void menu_port_clear(void);
void menu_port_draw_text (uint8_t col, uint8_t row, const char *s, menu_style_e style);
void menu_port_draw_int  (uint8_t col, uint8_t row, int32_t  v, uint8_t width, menu_style_e style);
void menu_port_draw_uint (uint8_t col, uint8_t row, uint32_t v, uint8_t width, menu_style_e style);
void menu_port_draw_float(uint8_t col, uint8_t row, float v, uint8_t int_w, uint8_t dec_w, menu_style_e style);

// Low-level pixel drawing (used by image_proc debug overlay). Bresenham line.
void menu_port_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color);

//--------------------------------------------------------------------------------------------------------------------
// Input : returns at most one event per call (priority ENTER > BACK > UP > DOWN).
//--------------------------------------------------------------------------------------------------------------------
void menu_port_scan_keys(menu_key_event_t *ev);

//--------------------------------------------------------------------------------------------------------------------
// Flash : one fixed page (matches the official SeekFree EEPROM demo). Data is an array of 32-bit words.
//--------------------------------------------------------------------------------------------------------------------
uint8_t menu_port_flash_write(const uint32_t *buf, uint16_t count);   // erase+write; returns 1 on success
void    menu_port_flash_read (uint32_t *buf, uint16_t count);         // read `count` words into buf

//--------------------------------------------------------------------------------------------------------------------
// Camera : blit the shared MT9V03X image, binarized at `threshold`, to the display.
//          The 188x120 source is scaled to fit.
//--------------------------------------------------------------------------------------------------------------------
void menu_port_show_gray(uint8_t threshold);

#endif
