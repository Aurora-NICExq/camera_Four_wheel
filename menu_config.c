/* menu_config.c */
#include "menu.h"
#include "config.h"

extern volatile float   steer_kp_min;
extern volatile float   steer_kp_max;
extern volatile float   steer_kp_e_sat;
extern volatile float   steer_kd;
extern volatile float   steer_d_filt_alpha;
extern volatile float   curve_cut;
extern volatile uint16_t straight_duty;
extern volatile int16_t straight_judge;
extern volatile int16_t straight_judge_13;
extern volatile int16_t image_threshold;
extern volatile uint8_t  drive_armed;
extern volatile uint16_t drive_duty_base;

const menu_item_t menu_items[] = {
    MENU_BOOL("Armed",      drive_armed,        0),
    MENU_F32("Kp Min",       steer_kp_min,       0.0f, 20.0f, 0.1f,  KP_MIN),
    MENU_F32("Kp Max",       steer_kp_max,       0.0f, 20.0f, 0.1f,  KP_MAX),
    MENU_F32("Kp E Sat",     steer_kp_e_sat,     5.0f, 80.0f, 1.0f,  KP_E_SAT),
    MENU_F32("Kd",           steer_kd,           0.0f, 30.0f, 0.1f,  KD),
    MENU_F32("D Filt Alpha", steer_d_filt_alpha, 0.0f, 1.0f,  0.05f, D_FILT_ALPHA),
    MENU_F32("Curve Cut",    curve_cut,          0.0f, 3000.0f, 50.0f, CURVE_CUT_DUTY),
    MENU_U16("Str Duty",    straight_duty,      0,    DUTY_HARD_CAP, 100, STRAIGHT_MAX_DUTY),
    MENU_I16("Threshold",    image_threshold,    0,    255,   1,     0),
    MENU_U16("Duty",        drive_duty_base,    0,    DUTY_HARD_CAP, 100, STRAIGHT_DUTY),
    MENU_I16("St Judge",    straight_judge,     1,    30,    1,     STRAIGHT_JUDGE),
    MENU_ACTION("Reset",        menu_action_reset),
    MENU_ACTION("Camera",       menu_action_camera),
    MENU_ACTION("Align Test",   menu_action_align_test),
    MENU_ACTION("Motor Test",   menu_action_motor_test),
    MENU_ACTION("Restore Def",  menu_action_defaults),
};
const uint16_t menu_item_count = (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));
