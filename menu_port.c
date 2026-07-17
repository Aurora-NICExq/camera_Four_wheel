/*********************************************************************************************************************
 * menu_port.c  --  SeekFree TC264 implementation of the menu hardware interface : 2.0" display (240x320).
 *                  THIS IS THE ONLY FILE THAT INCLUDES SEEKFREE HEADERS.
 *===================================================================================================================
 *  OLED-specific notes
 *  - Resolution: 240x320 pixels → 40 cols × 40 rows with 6x8 font
 *  - Cursor is shown with a ">" prefix and title decorations
 *===================================================================================================================
 *  INTEGRATION (3 edits to your project)
 *-------------------------------------------------------------------------------------------------------------------
 *  1) cpu0_main.c  (the display / menu core)
 *          #include "menu.h"
 *          ...
 *          clock_init();
 *          debug_init();
 *          menu_init();                       // sets up display + keys + 10 ms PIT, loads flash or defaults
 *          cpu_wait_event_ready();
 *          while (TRUE) { menu_task(); }       // non-blocking, call as fast as the loop runs
 *
 *  2) isr.c  --  inside the existing CCU60_CH0 handler, call the menu tick (this runs key_scanner()):
 *          #include "menu_port.h"
 *          IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
 *          {
 *              interrupt_global_enable(0);
 *              menu_port_pit_10ms_isr();       // <-- add this line (key_scanner + ms tick)
 *              pit_clear_flag(CCU60_CH0);
 *          }
 *
 *  3) cpu1_main.c  (the control loop) : owns the tunable globals (declared extern in menu_config.c) as
 *          volatile and updates the Monitor globals (encoder_left/right, line_error, loop_time_us).
 *          No other changes are needed -- the menu writes tunables straight into these volatiles
 *          (committed only when an edit finishes).
 *
 *  Note on cores: menu_port_init() runs on CPU0, so the CCU60_CH0 interrupt is serviced on CPU0 (same core
 *  as menu_task), and the ms tick is a CPU0-local variable -- no cross-core hazard on the time base.
 ********************************************************************************************************************/
#include "zf_common_headfile.h"     /* umbrella: oled, key, flash, pit, mt9v03x, zf types */
#include "menu_port.h"

//====================================================================================================================
// Configuration
//====================================================================================================================
#define MENU_FLASH_SECTOR   (0)      // matches the official SeekFree TC264 EEPROM demo (E08): sector 0 ...
#define MENU_FLASH_PAGE     (8)      // ... page 8   (DFLASH has 12 pages; pick a page nothing else uses)
#define KEY_REPEAT_MS       (120)    // long-press auto-repeat period (after the library's 1 s long-press threshold)

//====================================================================================================================
// Millisecond time base -- advanced by the 10 ms PIT ISR
//====================================================================================================================
static volatile uint32 s_tick_ms = 0;

void menu_port_pit_10ms_isr(void)
{
    key_scanner();          // debounce / classify the 4 buttons (10 ms scan period)
    s_tick_ms += 10;        // menu time base (10 ms resolution)
}

uint32_t menu_port_millis(void)
{
    return (uint32_t)s_tick_ms;
}

//====================================================================================================================
// Init
//====================================================================================================================
void menu_port_init(void)
{
    // Display : 2.0", 240x320.
    // Use 6x8 font to get 40 cols × 40 rows (240/6=40, 320/8=40).
    oled_set_font(OLED_6X8_FONT);
    oled_init();
    oled_clear();

    // Keys : scan period MUST equal the PIT period (10 ms) so long-press timing is correct.
    key_init(10);

    // 10 ms periodic interrupt on CCU60_CH0. Remember to call menu_port_pit_10ms_isr() from cc60_pit_ch0_isr.
    pit_ms_init(CCU60_CH0, 10);
}

//====================================================================================================================
// Display — monochrome OLED.  Character coordinates (col, row) are converted to pixel coordinates.
//   col → x = col * 6      (6x8 font, 6 pixels per character wide)
//   row → y = row * 8      (8 pixels per character tall)
//
//  Style mapping (monochrome — no colour):
//    NORMAL   : plain text, white on black
//    SELECTED : plain text, but the engine's label already includes ">" at col 0 for cursor indication
//    EDIT     : same as SELECTED; the engine draws the title bar with EDIT prefix
//    TITLE    : same as NORMAL; the engine decorates the title (e.g. "== NAME ==")
//
//  The engine's build_label() and draw_title*() functions handle all visual distinction. The port
//  layer simply draws the text as given at the requested position.
//====================================================================================================================

void menu_port_clear(void)
{
    oled_clear();
}

void menu_port_draw_text(uint8_t col, uint8_t row, const char *s, menu_style_e style)
{
    (void)style;   // monochrome: style differences are handled at the engine level (label markup)
    oled_show_string((uint16)(col * 6), (uint16)(row * 8), s);
}

void menu_port_draw_int(uint8_t col, uint8_t row, int32_t v, uint8_t width, menu_style_e style)
{
    // oled_show_int draws num + 1 columns (extra slot for sign), so to fill exactly
    // `width` columns pass num = width - 1.
    uint8_t num = (width > 1) ? (uint8_t)(width - 1) : 1;
    if (num > 10) num = 10;
    (void)style;
    oled_show_int((uint16)(col * 6), (uint16)(row * 8), v, num);
}

void menu_port_draw_uint(uint8_t col, uint8_t row, uint32_t v, uint8_t width, menu_style_e style)
{
    uint8_t num = (width < 1) ? 1 : (width > 10 ? 10 : (uint8_t)width);
    (void)style;
    oled_show_uint((uint16)(col * 6), (uint16)(row * 8), v, num);
}

void menu_port_draw_float(uint8_t col, uint8_t row, float v, uint8_t int_w, uint8_t dec_w, menu_style_e style)
{
    (void)style;
    oled_show_float((uint16)(col * 6), (uint16)(row * 8), (double)v, (uint8)int_w, (uint8)dec_w);
}

// Bresenham line algorithm — used by image_proc debug overlay.
void menu_port_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color)
{
    int16_t dx  = (int16_t)((x1 > x0) ? (x1 - x0) : (x0 - x1));
    int16_t dy  = (int16_t)((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int16_t sx  = (x0 < x1) ? 1 : -1;
    int16_t sy  = (y0 < y1) ? 1 : -1;
    int16_t err = (dx > dy ? dx : -dy) / 2;
    int16_t e2;

    for (;;)
    {
        oled_draw_point(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 = (uint16_t)((int16_t)x0 + sx); }
        if (e2 <  dy) { err += dx; y0 = (uint16_t)((int16_t)y0 + sy); }
    }
}

//====================================================================================================================
// Input
//   KEY_1=UP  KEY_2=DOWN  KEY_3=ENTER  KEY_4=BACK
//   ENTER/BACK act on a short press only. UP/DOWN: short press = one 1x event; a held key produces throttled
//   repeat events (is_repeat=1 -> engine applies 10x step). The library latches KEY_LONG_PRESS while held.
//====================================================================================================================
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
        uint32 now = s_tick_ms;
        if ((uint32)(now - last_repeat_ms) >= KEY_REPEAT_MS)     // throttle held-key repeats
        {
            last_repeat_ms = now;
            ev->key = (up == KEY_LONG_PRESS) ? MENU_KEY_UP : MENU_KEY_DOWN;
            ev->is_repeat = 1;
        }
        // do NOT clear the long-press state: it auto-latches while held; releasing resets it in the driver.
    }
}

//====================================================================================================================
// Flash  (fixed DFLASH page ; data is an array of 32-bit words, stored via the union buffer)
//====================================================================================================================
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

void menu_port_show_gray(uint8_t threshold)
{
    oled_show_gray_image(0, 0, mt9v03x_image[0],
                         MT9V03X_W, MT9V03X_H,   // source size: 188 × 120
                         240, 320,               // display size: scale to fit 240 × 320
                         (uint8)threshold);
}
