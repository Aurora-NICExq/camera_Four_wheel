# menu_port.h 逐行注释

> 行号与源文件一致。

菜单硬件抽象层（HAL）头文件：定义 IPS200 竖屏字符网格尺寸、显示样式、按键枚举与绘制/扫描 API。`menu.c` 仅通过本接口访问屏与键，便于仿真或换板时替换实现。

```c
// L1: 文件头注释
/* menu_port.h */
// L2: 头文件保护开始
#ifndef _menu_port_h_
// L3: 定义保护宏
#define _menu_port_h_
// L4: 空行

// L5: 标准整数类型
#include <stdint.h>
// L6: 布尔类型
#include <stdbool.h>
// L7: 空行

// L8: 竖屏每行 30 个字符（240px / 8px 字宽）
#define MENU_COLS   (30)       // 240 / 8 = 30  (竖屏宽)
// L9: 竖屏共 20 行（320px / 16px 字高）
#define MENU_ROWS   (20)       // 320 / 16 = 20 (竖屏高)
// L10: 空行

// L11: 文本显示样式枚举
typedef enum
// L12: 左花括号
{
// L13: 普通未选中行
    MENU_STYLE_NORMAL,
// L14: 列表当前光标行
    MENU_STYLE_SELECTED,
// L15: 编辑模式下行与数值
    MENU_STYLE_EDIT,
// L16: 顶栏标题行
    MENU_STYLE_TITLE,
// L17: 枚举结束
} menu_style_e;
// L18: 空行

// L19: 逻辑按键枚举：NONE 表示本帧无事件
typedef enum { MENU_KEY_NONE, MENU_KEY_UP, MENU_KEY_DOWN, MENU_KEY_ENTER, MENU_KEY_BACK } menu_key_e;
// L20: 空行

// L21: 单次按键事件结构
typedef struct
// L22: 左花括号
{
// L23: 哪个键
    menu_key_e key;
// L24: 是否为长按自动重复（仅 UP/DOWN 在编辑/列表中有效）
    uint8_t    is_repeat;
// L25: 结构体结束
} menu_key_event_t;
// L26: 空行

// L27: 初始化 IPS200 屏、背光、按键 GPIO
void     menu_port_init(void);
// L28: 毫秒时间戳，供消抖与长按判定
uint32_t menu_port_millis(void);
// L29: 周期性调用，推进按键 FSM 消抖（非阻塞）
void     menu_port_key_scan(void);
// L30: 空行

// L31: 清屏
void menu_port_clear(void);
// L32: 在字符网格 (col,row) 绘制字符串
void menu_port_draw_text (uint8_t col, uint8_t row, const char *s, menu_style_e style);
// L33: 绘制有符号整数，width 控制显示位数
void menu_port_draw_int  (uint8_t col, uint8_t row, int32_t  v, uint8_t width, menu_style_e style);
// L34: 绘制无符号整数
void menu_port_draw_uint (uint8_t col, uint8_t row, uint32_t v, uint8_t width, menu_style_e style);
// L35: 绘制浮点，int_w 整数位宽、dec_w 小数位宽
void menu_port_draw_float(uint8_t col, uint8_t row, float v, uint8_t int_w, uint8_t dec_w, menu_style_e style);
// L36: 空行

// L37: 从按键队列取一个事件；无事件时 key=MENU_KEY_NONE
void menu_port_scan_keys(menu_key_event_t *ev);
// L38: 空行

// L39: 结束头文件保护
#endif
```
