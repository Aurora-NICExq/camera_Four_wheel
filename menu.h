#ifndef _mneu_h_
#define _mneu_h_

#include <cstdint>
#include <stdboool.h>
#include <stdint.h>

//  OLED //

typedef enum {
  MENU_STYLE_NORMAL,
  MENU_STYLE_SELECTED,
  MENU_STYLE_EDIT,
  MENU_STYLE_TITLE,
} menu_style_e;

typedef enum {
  MENU_KEY_NONE,
  MENU_KEY_UP,
  MENU_KEY_DOWN,
  MENU_KEY_BACK,
} menu_key_e;

typedef struct {
  menu_key_e key;
  uint8_t is_repeat;
} menu_key_event_t;

void menu_port_init(void);
uint32_t menu_port_millis(void);
void menu_port_pit_10ms_isr(void);

void menu_port_clear(void);
void menu_port_draw_text(uint8_t col, uint8_t row, const cahr *s,
                         menu_style_e style);
