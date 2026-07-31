# menu_config.c 逐行注释

> 行号与源文件一致。

菜单表数据层：将 `config.h` 中的默认常量与各模块 `volatile` 调参变量绑定为 `menu_items[]` 数组。数值项顺序在前，动作项（含 Race Preset）在后。

```c
// L1: 文件头注释
/* menu_config.c */
// L2: 引入菜单类型与宏定义
#include "menu.h"
// L3: 引入工程默认常量（KP、KD、STEER_LOOK_FAR_DEFAULT 等）
#include "config.h"
// L4: 空行

// L5: 转向 P 增益，定义于 control.c
extern volatile float   steer_kp;
// L6: 转向 D 增益
extern volatile float   steer_kd;
// L7: D 项低通滤波系数 alpha
extern volatile float   steer_d_filt_alpha;
// L8: 图像二值化阈值
extern volatile int16_t image_threshold;
// L9: 十字补线开关
extern volatile uint8_t image_cross_fill;
// L10: 前瞻滑窗起点行（Look Far）
extern volatile uint16_t steer_look_far;
// L11: 发车武装开关（菜单 Armed）
extern volatile uint8_t  drive_armed;
// L12: 武装后自动停车倒计时（秒）
extern volatile uint16_t drive_stop_time_s;
// L13: 直行基准占空比
extern volatile uint16_t drive_duty_base;
// L14: 空行

// L15: 菜单项数组开始；顺序即列表显示顺序
const menu_item_t menu_items[] = {
// L16: Armed：默认关；主循环据此决定是否允许发车
    MENU_BOOL("Armed",      drive_armed,        0),
// L17: Fine Step：细调步长开关，默认关
    MENU_BOOL("Fine Step",  menu_fine_step,     0),
// L18: Kp：0~20，步进 0.1，默认取 config.h 的 KP
    MENU_F32("Kp",           steer_kp,           0.0f, 20.0f, 0.1f,  KP),
// L19: Kd：0~30，步进 0.1，默认 KD
    MENU_F32("Kd",           steer_kd,           0.0f, 30.0f, 0.1f,  KD),
// L20: D 滤波 alpha：0~1，步进 0.05
    MENU_F32("D Filt Alpha", steer_d_filt_alpha, 0.0f, 1.0f,  0.05f, D_FILT_ALPHA),
// L21: 阈值：0~255，整型步进 1；默认 0 表示 OTSU/自动
    MENU_I16("Threshold",    image_threshold,    0,    255,   1,     0),
// L22: 十字补线：默认开
    MENU_BOOL("Cross Fill",  image_cross_fill,   1),
// L23: Look Far：合法范围 [span+1, MAX]，步进 5，默认 STEER_LOOK_FAR_DEFAULT
    MENU_U16("Look Far",     steer_look_far,     STEER_LOOK_SPAN + 1, STEER_LOOK_FAR_MAX, 5, STEER_LOOK_FAR_DEFAULT),
// L24: Duty：0~DUTY_HARD_CAP，步进 100，默认 STRAIGHT_DUTY
    MENU_U16("Duty",        drive_duty_base,    0,    DUTY_HARD_CAP, 100, STRAIGHT_DUTY),
// L25: Stop Time：1~600 秒，武装超时停车
    MENU_U16("Stop Time",   drive_stop_time_s,  1,    600,   1,     DRIVE_ARMED_TIMEOUT_S),
// L26: 进入三档 Race Preset 子菜单（Low/Mid/High）
    MENU_ACTION("Race Preset",  menu_action_race_preset),
// L27: 重置 control 与 motor 状态
    MENU_ACTION("Reset",        menu_action_reset),
// L28: 全屏摄像头预览
    MENU_ACTION("Camera",       menu_action_camera),
// L29: 对齐测试（停电机 + 摄像头）
    MENU_ACTION("Align Test",   menu_action_align_test),
// L30: 双电机 20% 占空比测试
    MENU_ACTION("Motor Test",   menu_action_motor_test),
// L31: 仅左轮 PWM 测试，用于确认左右接线
    MENU_ACTION("Left Test",    menu_action_left_test),
// L32: 恢复所有数值项为 def，不应用 Race Preset
    MENU_ACTION("Restore Def",  menu_action_defaults),
// L33: 数组结束
};
// L34: 编译期计算菜单项数量，供滚动与边界判断
const uint16_t menu_item_count = (uint16_t)(sizeof(menu_items) / sizeof(menu_items[0]));
```
