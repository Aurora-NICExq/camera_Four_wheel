#ifndef _menu_port_h_
#define _menu_port_h_

#include <stdint.h>
#include <stdbool.h>


#define MENU_COLS   (30)
#define MENU_ROWS   (20)


typedef enum
{
    MENU_STYLE_NORMAL,
    MENU_STYLE_SELECTED,
    MENU_STYLE_EDIT,
    MENU_STYLE_TITLE,
} menu_style_e;


typedef enum { MENU_KEY_NONE, MENU_KEY_UP, MENU_KEY_DOWN, MENU_KEY_ENTER, MENU_KEY_BACK } menu_key_e;

typedef struct
{
    menu_key_e key;
    uint8_t    is_repeat;
} menu_key_event_t;


void     menu_port_init(void);
uint32_t menu_port_millis(void);


void     menu_port_key_scan(void);


void menu_port_clear(void);
void menu_port_draw_text (uint8_t col, uint8_t row, const char *s, menu_style_e style);
void menu_port_draw_int  (uint8_t col, uint8_t row, int32_t  v, uint8_t width, menu_style_e style);
void menu_port_draw_uint (uint8_t col, uint8_t row, uint32_t v, uint8_t width, menu_style_e style);
void menu_port_draw_float(uint8_t col, uint8_t row, float v, uint8_t int_w, uint8_t dec_w, menu_style_e style);


void menu_port_scan_keys(menu_key_event_t *ev);


void menu_port_draw_key_status(void);
uint8_t menu_port_key_pressed(menu_key_e key);

#endif
