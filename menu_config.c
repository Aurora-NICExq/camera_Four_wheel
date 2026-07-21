/* menu_config.c */
#include "menu.h"
#include "config.h"

extern volatile float   steer_kp_min;
extern volatile float   steer_kp_max;
extern volatile float   steer_kp_e_sat;
extern volatile float   steer_kd;
extern volatile float   steer_d_filt_alpha;
extern volatile int16_t image_threshold;

const menu_item_t menu_items[] = {
    MENU_F32("Kp Min",       steer_kp_min,       0.0f, 20.0f, 0.1f,  KP_MIN),
    MENU_F32("Kp Max",       steer_kp_max,       0.0f, 20.0f, 0.1f,  KP_MAX),
    MENU_F32("Kp E Sat",     steer_kp_e_sat,     5.0f, 80.0f, 1.0f,  KP_E_SAT),
    MENU_F32("Kd",           steer_kd,           0.0f, 30.0f, 0.1f,  KD),
    MENU_F32("D Filt Alpha", steer_d_filt_alpha, 0.0f, 1.0f,  0.05f, D_FILT_ALPHA),
    MENU_I16("Threshold",    image_threshold,    0,    255,   1,     0),
    MENU_ACTION("Save",         menu_action_save),
    MENU_ACTION("Load",         menu_action_load),
    MENU_ACTION("Restore Def",  menu_action_defaults),
};
const uint16_t menu_item_count = (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));
