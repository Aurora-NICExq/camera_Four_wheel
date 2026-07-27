/* menu_config.c */
#include "menu.h"
#include "config.h"

extern volatile float    steer_kp;
extern volatile float    steer_kp2;
extern volatile float    steer_ki;
extern volatile float    steer_kd;
extern volatile float    steer_ka;
extern volatile float    steer_up;
extern volatile float    speed_up;
extern volatile int16_t  straight_judge;
extern volatile int16_t  straight_judge_13;
extern volatile int16_t  image_threshold;
extern volatile uint8_t  drive_armed;
extern volatile uint16_t drive_duty_base;

const menu_item_t menu_items[] = {
    MENU_BOOL("Armed",      drive_armed,        0),
    MENU_F32("Servo P",     steer_kp,           0.0f, 30.0f, 0.1f,  SERVO_P),
    MENU_F32("Servo P2",    steer_kp2,          0.0f, 5.0f,  0.05f, SERVO_P2),
    MENU_F32("Servo D",     steer_kd,           0.0f, 30.0f, 0.1f,  SERVO_D),
    MENU_F32("Steer Up",    steer_up,           0.0f, 3000.0f, 50.0f, STEER_UP_DUTY),
    MENU_F32("Speed Up",    speed_up,           0.0f, 3000.0f, 50.0f, SPEED_UP_DUTY),
    MENU_I16("Threshold",   image_threshold,    0,    255,   1,     0),
    MENU_U16("Ex Speed",    drive_duty_base,    0,    DUTY_HARD_CAP, 100, EX_SPEED_DUTY),
    MENU_I16("St Judge",    straight_judge,   1,    30,    1,     STRAIGHT_JUDGE),
    MENU_ACTION("Reset",        menu_action_reset),
    MENU_ACTION("Camera",       menu_action_camera),
    MENU_ACTION("Align Test",   menu_action_align_test),
    MENU_ACTION("Motor Test",   menu_action_motor_test),
    MENU_ACTION("Restore Def",  menu_action_defaults),
};
const uint16_t menu_item_count = (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));
