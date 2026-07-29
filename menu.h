/* menu.h - menu API + item types */
#ifndef _menu_h_
#define _menu_h_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    ITEM_INT16,     // -> volatile int16_t
    ITEM_UINT16,    // -> volatile uint16_t
    ITEM_FLOAT,     // -> volatile float
    ITEM_BOOL,      // -> volatile uint8_t (0/1)
    ITEM_ACTION,    // 调 action()，无变量
} item_type_e;

/* 一行 = 一个可调项 / 动作。min/max/step/def 均为 float，按 type 转型 —— 单结构覆盖所有数值类型。
 * 短按 = 1×step，长按自动重复 = 10×step。 */
typedef struct
{
    const char  *name;
    void        *var;               // &全局；ACTION 项为 NULL
    item_type_e  type;
    float        min, max, step, def;
    void       (*action)(void);     // 仅 ACTION 项
} menu_item_t;

#define MENU_F32(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_FLOAT , (mn), (mx), (st), (df), 0 }
#define MENU_I16(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_INT16 , (mn), (mx), (st), (df), 0 }
#define MENU_U16(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_UINT16, (mn), (mx), (st), (df), 0 }
#define MENU_BOOL(nm, v, df)             { (nm), (void*)&(v), ITEM_BOOL  , 0, 1, 1, (df), 0 }
#define MENU_ACTION(nm, fn)              { (nm), 0, ITEM_ACTION, 0, 0, 0, 0, (fn) }

extern volatile uint8_t  menu_fine_step;  /* 1 = 数值项按细步进调整 */

extern const menu_item_t menu_items[];
extern const uint16_t    menu_item_count;

void menu_init(void);   // CPU0 上调一次：显示 + 按键 + 默认参数
void menu_task(void);   // 主循环每帧调用；非阻塞

void menu_action_defaults(void);
void menu_action_race_preset(void);
void menu_action_cam_calib(void);
void menu_action_camera(void);
void menu_action_align_test(void);
void menu_action_motor_test(void);
void menu_action_reset(void);

uint8_t menu_camera_view(void);
uint8_t menu_calib_view(void);
uint8_t menu_align_test_mode(void);
uint8_t menu_motor_test_mode(void);

#endif
