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
// cpu0_main.c / control.c) and MUST be `volatile`.
//
// For quick MENU-ONLY testing without any control code, set
// MENU_STANDALONE_TEST to 1 to define them here instead. Set it back to 0 for
// the real car (control code owns them).
//====================================================================================================================
#define MENU_STANDALONE_TEST (1)

#if MENU_STANDALONE_TEST
volatile float   steer_kp_min        = 1.0f;
volatile float   steer_kp_max        = 3.2f;
volatile float   steer_kp_e_sat      = 40.0f;
volatile uint8_t steer_use_const_kp  = 0;
volatile float   steer_kp_const      = 1.8f;
volatile float   steer_kd            = 6.0f;
volatile float   steer_d_filt_alpha  = 0.4f;
volatile uint16_t speed_straight_duty = 4500;
volatile uint16_t speed_hard_cap     = 6000;
volatile uint16_t speed_min_turn     = 2600;
volatile uint16_t speed_boost_duty   = 5200;
volatile uint16_t speed_slew_up      = 120;
volatile int16_t  image_threshold    = 128;
volatile uint16_t servo_center       = 750;
volatile uint16_t servo_range        = 110;

volatile int16_t  mon_error          = 0;
volatile uint16_t mon_duty           = 0;
volatile uint16_t mon_valid_rows     = 0;
volatile uint32_t mon_proc_us        = 0;
#else
extern volatile float   steer_kp_min;
extern volatile float   steer_kp_max;
extern volatile float   steer_kp_e_sat;
extern volatile uint8_t steer_use_const_kp;
extern volatile float   steer_kp_const;
extern volatile float   steer_kd;
extern volatile float   steer_d_filt_alpha;
extern volatile uint16_t speed_straight_duty;
extern volatile uint16_t speed_hard_cap;
extern volatile uint16_t speed_min_turn;
extern volatile uint16_t speed_boost_duty;
extern volatile uint16_t speed_slew_up;
extern volatile int16_t  image_threshold;
extern volatile uint16_t servo_center;
extern volatile uint16_t servo_range;

extern volatile int16_t  mon_error;
extern volatile uint16_t mon_duty;
extern volatile uint16_t mon_valid_rows;
extern volatile uint32_t mon_proc_us;
#endif

//====================================================================================================================
// Pages  (order MUST match the menu_page_e enum in menu.h)
//====================================================================================================================
const menu_page_t menu_pages[PAGE_NUM] = {
    {"SteerPID", PAGE_KIND_NORMAL},
    {"Speed",    PAGE_KIND_NORMAL},
    {"Image",    PAGE_KIND_NORMAL},
    {"Servo",    PAGE_KIND_NORMAL},
    {"Monitor",  PAGE_KIND_MONITOR},
    {"System",   PAGE_KIND_NORMAL},
};

//====================================================================================================================
// Items  (one row == one tunable / action ; grouped by page for readability,
// but order is free)
//====================================================================================================================
const menu_item_t menu_items[] = {
    // ---- SteerPID : direction loop ----
    MENU_F32("Kp Min",     steer_kp_min,       0.0f, 20.0f,  0.1f,  1.0f,  PAGE_STEER_PID),
    MENU_F32("Kp Max",     steer_kp_max,       0.0f, 20.0f,  0.1f,  3.2f,  PAGE_STEER_PID),
    MENU_F32("Kp E Sat",   steer_kp_e_sat,     5.0f, 80.0f,  1.0f,  40.0f, PAGE_STEER_PID),
    MENU_BOOL("Use Const Kp", steer_use_const_kp,     0,     PAGE_STEER_PID),
    MENU_F32("Kp Const",   steer_kp_const,     0.0f, 10.0f,  0.1f,  1.8f,  PAGE_STEER_PID),
    MENU_F32("Kd",         steer_kd,           0.0f, 30.0f,  0.1f,  6.0f,  PAGE_STEER_PID),
    MENU_F32("D Filt Alpha", steer_d_filt_alpha, 0.0f, 1.0f,  0.05f, 0.4f,  PAGE_STEER_PID),

    // ---- Speed ----
    MENU_U16("Straight",   speed_straight_duty, 1000, 10000, 50,  4500, PAGE_SPEED),
    MENU_U16("Hard Cap",   speed_hard_cap,      1000, 10000, 50,  6000, PAGE_SPEED),
    MENU_U16("Min Turn",   speed_min_turn,      1000, 10000, 50,  2600, PAGE_SPEED),
    MENU_U16("Boost",      speed_boost_duty,    1000, 10000, 50,  5200, PAGE_SPEED),
    MENU_U16("Slew Up",    speed_slew_up,       10,   1000,  10,  120,  PAGE_SPEED),

    // ---- Image ----
    MENU_I16("Threshold", image_threshold, 0, 255, 1, 128, PAGE_IMAGE),

    // ---- Servo ----
    MENU_U16("Center", servo_center, 500, 1000, 1, 750, PAGE_SERVO),
    MENU_U16("Range",  servo_range,  30,  200,  1, 110, PAGE_SERVO),

    // ---- System : actions only ----
    MENU_ACTION("Save",         menu_action_save,     PAGE_SYSTEM),
    MENU_ACTION("Load",         menu_action_load,     PAGE_SYSTEM),
    MENU_ACTION("Restore Def",  menu_action_defaults, PAGE_SYSTEM),
};
const uint16_t menu_item_count =
    (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));

//====================================================================================================================
// Monitor fields  (live, read-only ; label + value, refreshed ~10 Hz)
//====================================================================================================================
const monitor_field_t menu_monitor_fields[] = {
    {"Error",     (const void *)&mon_error,      MON_I16},
    {"Duty",      (const void *)&mon_duty,       MON_U16},
    {"ValidRows", (const void *)&mon_valid_rows, MON_U16},
    {"Proc us",   (const void *)&mon_proc_us,    MON_U32},
};
const uint16_t menu_monitor_count =
    (uint16_t)(sizeof(menu_monitor_fields) / sizeof(menu_monitor_fields[0]));
