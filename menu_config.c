/*********************************************************************************************************************
 * menu_config.c  --  THE ONLY place menu items / pages / monitor fields are
 * defined.
 *===================================================================================================================
 *  README : how to extend the menu
 *-------------------------------------------------------------------------------------------------------------------
 *  ADD A PARAMETER
 *      Add ONE line to menu_items[] on the page you want, using a MENU_* macro:
 *          MENU_F32("Kd", steer_kd, 0.0f, 50.0f, 0.1f, 1.2f, PAGE_STEER_PID),
 * // float MENU_I16("Base", base_speed, 0, 3000, 10, 800, PAGE_SPEED),  //
 * int16 MENU_U16("Foo",  foo_u16,    0, 6000, 5,  100, PAGE_SPEED),          //
 * uint16 MENU_BOOL("Motor En", motor_enable, 1, PAGE_SPEED),  // bool toggle
 * (0/1) Args are: ("Name", your_global, min, max, step, default, PAGE_xxx).
 *      Short press = 1x step ; long press = 10x step (auto-repeat).
 *      Make sure `your_global` is declared (extern) below and is `volatile` in
 * your control code.
 *      >>> Then BUMP MENU_FLASH_VERSION in menu.h so a saved-but-incompatible
 * flash image is ignored. <<<
 *
 *  ADD A PAGE
 *      1) add an enum value in menu.h  (menu_page_e, BEFORE PAGE_NUM)
 *      2) add one row to menu_pages[] below, in the SAME order  ({ "Name",
 * PAGE_KIND_NORMAL }).
 *
 *  ADD AN ACTION ITEM
 *      Write a `void my_action(void)` (here or anywhere), then add:
 *          MENU_ACTION("Do It", my_action, PAGE_SYSTEM),
 *      The engine already provides: menu_action_save / menu_action_load /
 * menu_action_defaults -- use them directly as shown in the System page.
 *
 *  NOTE ON TYPES
 *      int16_t == SeekFree int16, uint16_t == uint16, uint8_t == uint8,
 * uint32_t == uint32. The extern names below MUST match YOUR existing globals
 * -- rename them to yours. Because the menu only takes their addresses, your
 * control code on CPU1 needs zero changes.
 ********************************************************************************************************************/
#include "menu.h"

//====================================================================================================================
// Your tunable globals.  These live in your control code (typically
// cpu1_main.c) and MUST be `volatile`.
//
// For quick MENU-ONLY testing without any control code, set
// MENU_STANDALONE_TEST to 1 to define them here instead. Set it back to 0 for
// the real car (control code owns them).
//====================================================================================================================
#define MENU_STANDALONE_TEST (0)

#if MENU_STANDALONE_TEST
volatile float steer_kp = 3.5f, steer_kd = 1.2f;
volatile float speed_kp = 2.0f, speed_ki = 0.5f;
volatile int16_t base_speed = 800, max_speed = 1500;
volatile int16_t servo_center = 750, servo_left = 650, servo_right = 850;
volatile uint8_t motor_enable = 1;
volatile int16_t encoder_left = 0, encoder_right = 0, line_error = 0;
volatile uint32_t loop_time_us = 0;
#else
extern volatile float steer_kp;
extern volatile float steer_kd;
extern volatile float speed_kp;
extern volatile float speed_ki;
extern volatile int16_t base_speed;
extern volatile int16_t max_speed;
extern volatile int16_t servo_center;
extern volatile int16_t servo_left;
extern volatile int16_t servo_right;
extern volatile uint8_t motor_enable; // example bool

// Read-only values shown on the Monitor page (updated by the control loop on
// CPU1)
extern volatile int16_t encoder_left;
extern volatile int16_t encoder_right;
extern volatile int16_t line_error;
extern volatile uint32_t loop_time_us;
#endif

//====================================================================================================================
// Pages  (order MUST match the menu_page_e enum in menu.h)
//====================================================================================================================
const menu_page_t menu_pages[PAGE_NUM] = {
    {"SteerPID", PAGE_KIND_NORMAL}, {"SpeedPID", PAGE_KIND_NORMAL},
    {"Speed", PAGE_KIND_NORMAL},    {"Servo", PAGE_KIND_NORMAL},
    {"Monitor", PAGE_KIND_MONITOR}, {"System", PAGE_KIND_NORMAL},
};

//====================================================================================================================
// Items  (one row == one tunable / action ; grouped by page for readability,
// but order is free)
//====================================================================================================================
const menu_item_t menu_items[] = {
    // ---- SteerPID : direction loop ----
    MENU_F32("Kp", steer_kp, 0.0f, 50.0f, 0.1f, 3.5f, PAGE_STEER_PID),
    MENU_F32("Kd", steer_kd, 0.0f, 50.0f, 0.1f, 1.2f, PAGE_STEER_PID),

    // ---- SpeedPID ----
    MENU_F32("Kp", speed_kp, 0.0f, 50.0f, 0.1f, 2.0f, PAGE_SPEED_PID),
    MENU_F32("Ki", speed_ki, 0.0f, 50.0f, 0.05f, 0.5f, PAGE_SPEED_PID),

    // ---- Speed ----
    MENU_I16("Base", base_speed, 0, 3000, 10, 800, PAGE_SPEED),
    MENU_I16("Max", max_speed, 0, 5000, 10, 1500, PAGE_SPEED),
    MENU_BOOL("Motor En", motor_enable, 1, PAGE_SPEED),

    // ---- Servo ----
    MENU_I16("Center", servo_center, 500, 1000, 1, 750, PAGE_SERVO),
    MENU_I16("Left Lim", servo_left, 500, 1000, 1, 650, PAGE_SERVO),
    MENU_I16("Right Lim", servo_right, 500, 1000, 1, 850, PAGE_SERVO),

    // ---- System : actions only ----
    MENU_ACTION("Save", menu_action_save, PAGE_SYSTEM),
    MENU_ACTION("Load", menu_action_load, PAGE_SYSTEM),
    MENU_ACTION("Restore Def", menu_action_defaults, PAGE_SYSTEM),
};
const uint16_t menu_item_count =
    (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));

//====================================================================================================================
// Monitor fields  (live, read-only ; label + value, refreshed ~10 Hz)
//====================================================================================================================
const monitor_field_t menu_monitor_fields[] = {
    {"Enc L", (const void *)&encoder_left, MON_I16},
    {"Enc R", (const void *)&encoder_right, MON_I16},
    {"Error", (const void *)&line_error, MON_I16},
    {"Loop us", (const void *)&loop_time_us, MON_U32},
};
const uint16_t menu_monitor_count =
    (uint16_t)(sizeof(menu_monitor_fields) / sizeof(menu_monitor_fields[0]));
