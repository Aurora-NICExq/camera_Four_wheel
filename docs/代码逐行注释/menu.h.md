# menu.h 逐行注释

> 行号与源文件一致。

菜单模块公共头文件：定义菜单项类型、表项结构、初始化宏，以及 `menu_init` / `menu_task` 等对外接口。

```c
// L1: 文件头注释，标识本文件为 menu.h
/* menu.h */
// L2: 头文件保护宏开始，防止重复包含
#ifndef _menu_h_
// L3: 定义保护宏 _menu_h_
#define _menu_h_
// L4: 空行

// L5: 引入标准整数类型 uint8_t、uint16_t 等
#include <stdint.h>
// L6: 引入布尔类型 bool
#include <stdbool.h>
// L7: 空行

// L8: 菜单项数据类型枚举开始
typedef enum
// L9: 左花括号
{
// L10: 有符号 16 位整型参数（如 Threshold）
    ITEM_INT16,
// L11: 无符号 16 位整型参数（如 Look Far、Duty）
    ITEM_UINT16,
// L12: 浮点参数（如 Kp、Kd、D Filt Alpha）
    ITEM_FLOAT,
// L13: 布尔开关（0/1，如 Armed、Fine Step）
    ITEM_BOOL,
// L14: 动作项，无绑定变量，仅触发回调函数
    ITEM_ACTION,
// L15: 枚举结束
} item_type_e;
// L16: 空行

// L17: 单条菜单项描述结构体开始
typedef struct
// L18: 左花括号
{
// L19: 显示名称字符串指针
    const char  *name;
// L20: 绑定变量地址；ACTION 项为 NULL
    void        *var;
// L21: 变量类型，决定读写与显示方式
    item_type_e  type;
// L22: 可调范围下限、上限、步进、默认值（浮点统一表示）
    float        min, max, step, def;
// L23: ACTION 类型回调；数值项为 NULL
    void       (*action)(void);
// L24: 结构体结束
} menu_item_t;
// L25: 空行

// L26: 宏：声明一条 FLOAT 菜单项，参数为名称、变量、最小、最大、步进、默认
#define MENU_F32(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_FLOAT , (mn), (mx), (st), (df), 0 }
// L27: 宏：声明一条 INT16 菜单项
#define MENU_I16(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_INT16 , (mn), (mx), (st), (df), 0 }
// L28: 宏：声明一条 UINT16 菜单项
#define MENU_U16(nm, v, mn, mx, st, df)  { (nm), (void*)&(v), ITEM_UINT16, (mn), (mx), (st), (df), 0 }
// L29: 宏：声明 BOOL 项；min=0 max=1 step=1，仅 def 有意义
#define MENU_BOOL(nm, v, df)             { (nm), (void*)&(v), ITEM_BOOL  , 0, 1, 1, (df), 0 }
// L30: 宏：声明 ACTION 项，绑定无变量，action 字段为函数指针
#define MENU_ACTION(nm, fn)              { (nm), 0, ITEM_ACTION, 0, 0, 0, 0, (fn) }
// L31: 空行

// L32: 全局 Fine Step 开关，1 时编辑步长缩小 10 倍
extern volatile uint8_t  menu_fine_step;
// L33: 空行

// L34: 菜单表定义于 menu_config.c
extern const menu_item_t menu_items[];
// L35: 菜单项总数
extern const uint16_t    menu_item_count;
// L36: 空行

// L37: 初始化菜单与显示
void menu_init(void);
// L38: 主循环中周期性调用：扫描按键并驱动 FSM
void menu_task(void);
// L39: 空行

// L40: 动作：恢复所有数值项为 def 默认值
void menu_action_defaults(void);
// L41: 动作：进入 Race Preset 子页面（三档预设）
void menu_action_race_preset(void);
// L42: 动作：进入摄像头全屏预览
void menu_action_camera(void);
// L43: 动作：进入对齐测试（摄像头 + 停车）
void menu_action_align_test(void);
// L44: 动作：进入电机测试（固定 20% 占空比）
void menu_action_motor_test(void);
// L45: 动作：进入左轮 PWM 辨识测试
void menu_action_left_test(void);
// L46: 动作：重置控制与电机状态
void menu_action_reset(void);
// L47: 空行

// L48: 查询是否处于摄像头视图（1=是）
uint8_t menu_camera_view(void);
// L49: 查询是否处于对齐测试模式
uint8_t menu_align_test_mode(void);
// L50: 查询是否处于电机测试模式
uint8_t menu_motor_test_mode(void);
// L51: 查询是否处于左轮测试模式
uint8_t menu_left_test_mode(void);
// L52: 空行

// L53: 结束头文件保护
#endif
```
