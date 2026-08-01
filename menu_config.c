/* menu_config.c */
#include "menu.h"
#include "config.h"

extern volatile float   steer_kp;
extern volatile float   steer_kd;
extern volatile float   steer_d_filt_alpha;
extern volatile int16_t image_threshold;
extern volatile uint8_t image_cross_fill;
extern volatile uint16_t steer_look_far;
extern volatile uint8_t  drive_armed;
extern volatile uint16_t drive_stop_time_s;
extern volatile uint16_t drive_duty_base;
extern volatile uint16_t drive_failsafe_frames;

const menu_item_t menu_items[] = {
    MENU_BOOL("Armed",      drive_armed,        0),
    MENU_BOOL("Fine Step",  menu_fine_step,     0),
    MENU_F32("Kp",           steer_kp,           0.0f, 20.0f, 0.1f,  KP),
    MENU_F32("Kd",           steer_kd,           0.0f, 30.0f, 0.1f,  KD),
    MENU_F32("D Filt Alpha", steer_d_filt_alpha, 0.0f, 1.0f,  0.05f, D_FILT_ALPHA),
    MENU_I16("Threshold",    image_threshold,    0,    255,   1,     0),
    MENU_BOOL("Cross Fill",  image_cross_fill,   1),
    MENU_U16("Look Far",     steer_look_far,     STEER_LOOK_SPAN + 1, STEER_LOOK_FAR_MAX, 5, STEER_LOOK_FAR_DEFAULT),
    MENU_U16("Duty",        drive_duty_base,    0,    DUTY_HARD_CAP, 100, STRAIGHT_DUTY),
    MENU_U16("Stop Time",   drive_stop_time_s,  1,    600,   1,     DRIVE_ARMED_TIMEOUT_S),
    MENU_U16("Lost Fr",     drive_failsafe_frames, FAILSAFE_FRAMES_MIN, FAILSAFE_FRAMES_MAX, 1, FAILSAFE_FRAMES_DEFAULT),
    MENU_ACTION("Race Preset",  menu_action_race_preset),
    MENU_ACTION("Reset",        menu_action_reset),
    MENU_ACTION("Camera",       menu_action_camera),
    MENU_ACTION("Align Test",   menu_action_align_test),
    MENU_ACTION("Motor Test",   menu_action_motor_test),
    MENU_ACTION("Left Test",    menu_action_left_test),
    MENU_ACTION("Restore Def",  menu_action_defaults),
};
const uint16_t menu_item_count = (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));
