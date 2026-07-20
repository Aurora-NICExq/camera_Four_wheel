/*********************************************************************************************************************
 * menu_config.c — 菜单项表的唯一定义处（单层扁平列表）。
 *   加一项：往 menu_items[] 加一行 MENU_* 宏（变量须是 control.c/image.c 里的 volatile 全局，并在下方 extern），
 *           然后 bump menu.h 的 MENU_FLASH_VERSION。
 *   这些 volatile 全局就是控制/图像每帧读取的那一份 —— 菜单里改完下一帧即生效。
 ********************************************************************************************************************/
#include "menu.h"
#include "config.h"    // 默认值取自 config.h 宏，保持单一真源

// 控制/图像逻辑拥有的可调 volatile 全局
extern volatile float   steer_kp_min;
extern volatile float   steer_kp_max;
extern volatile float   steer_kp_e_sat;
extern volatile uint8_t steer_use_const_kp;
extern volatile float   steer_kp_const;
extern volatile float   steer_kd;
extern volatile float   steer_d_filt_alpha;
extern volatile int16_t image_threshold;    // 0 = 自动大津法，>0 = 手动固定阈值

extern void menu_action_arm(void);           // 解锁/上锁（cpu0_main.c，非持久）

const menu_item_t menu_items[] = {
    MENU_F32("Kp Min",       steer_kp_min,       0.0f, 20.0f, 0.1f,  KP_MIN),
    MENU_F32("Kp Max",       steer_kp_max,       0.0f, 20.0f, 0.1f,  KP_MAX),
    MENU_F32("Kp E Sat",     steer_kp_e_sat,     5.0f, 80.0f, 1.0f,  KP_E_SAT),
    MENU_BOOL("Use Const Kp", steer_use_const_kp,             USE_CONST_KP),
    MENU_F32("Kp Const",     steer_kp_const,     0.0f, 10.0f, 0.1f,  KP_CONST),
    MENU_F32("Kd",           steer_kd,           0.0f, 30.0f, 0.1f,  KD),
    MENU_F32("D Filt Alpha", steer_d_filt_alpha, 0.0f, 1.0f,  0.05f, D_FILT_ALPHA),
    MENU_I16("Threshold",    image_threshold,    0,    255,   1,     0),
    MENU_ACTION("Arm/Disarm",   menu_action_arm),
    MENU_ACTION("Save",         menu_action_save),
    MENU_ACTION("Load",         menu_action_load),
    MENU_ACTION("Restore Def",  menu_action_defaults),
};
const uint16_t menu_item_count = (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));
