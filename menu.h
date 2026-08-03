/* menu.h */
#ifndef _menu_h_
#define _menu_h_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    ITEM_INT16,
    ITEM_UINT16,
    ITEM_FLOAT,
    ITEM_BOOL,
    ITEM_ACTION,
} item_type_e;

typedef struct
{
    const char  *name;
    void        *var;
    item_type_e  type;
    float        min, max, step, def;
    void       (*action)(void);
} menu_item_t;

#define MENU_F32(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_FLOAT , (mn), (mx), (st), (df), 0 }
#define MENU_I16(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_INT16 , (mn), (mx), (st), (df), 0 }
#define MENU_U16(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_UINT16, (mn), (mx), (st), (df), 0 }
#define MENU_BOOL(nm, v, df)             { (nm), (void*)&(v), ITEM_BOOL  , 0, 1, 1, (df), 0 }
#define MENU_ACTION(nm, fn)              { (nm), 0, ITEM_ACTION, 0, 0, 0, 0, (fn) }

extern volatile uint8_t  menu_fine_step;

extern const menu_item_t menu_items[];
extern const uint16_t    menu_item_count;

void menu_init(void);
void menu_task(void);

void menu_action_defaults(void);
void menu_action_race_preset(void);
void menu_action_camera(void);
void menu_action_align_test(void);
void menu_action_motor_test(void);
void menu_action_left_test(void);
void menu_action_reset(void);
void menu_action_dump_log(void);

uint8_t menu_camera_view(void);
uint8_t menu_align_test_mode(void);
uint8_t menu_motor_test_mode(void);
uint8_t menu_left_test_mode(void);

#endif
