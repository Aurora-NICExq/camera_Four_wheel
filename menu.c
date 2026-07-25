/* menu.c - data-driven tune menu engine */
#include "menu.h"
#include "menu_port.h"
#include "control.h"
#include "motor.h"

#define CONTENT_ROWS  (MENU_ROWS - 1)          // 第 0 行为标题
#define VALUE_COL     (18)                     // 竖屏 30 列：左侧标签，右侧数值
#define VALUE_WIDTH   (MENU_COLS - VALUE_COL)
#define KEY_STEP_MULT (10.0f)                   // 长按自动重复步长倍率

typedef enum { NAV_LIST, NAV_EDIT } nav_state_e;

static nav_state_e       s_nav;
static uint8_t           s_camera_view;
static uint8_t           s_align_test_mode;
static uint8_t           s_motor_test_mode;
static uint8_t           s_cursor;              // 选中项索引
static uint8_t           s_top;                 // 滚动窗口顶
static float             s_edit_val;            // 编辑工作副本
static const menu_item_t *s_edit_item;

static int32_t round_f(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

static float item_get(const menu_item_t *it)
{
    switch (it->type)
    {
    case ITEM_INT16:  return (float)(*(volatile int16_t  *)it->var);
    case ITEM_UINT16: return (float)(*(volatile uint16_t *)it->var);
    case ITEM_FLOAT:  return *(volatile float *)it->var;
    case ITEM_BOOL:   return (*(volatile uint8_t *)it->var) ? 1.0f : 0.0f;
    default:          return 0.0f;
    }
}

static void item_set(const menu_item_t *it, float v)
{
    if (v < it->min) v = it->min;
    if (v > it->max) v = it->max;
    switch (it->type)
    {
    case ITEM_INT16:  *(volatile int16_t  *)it->var = (int16_t)round_f(v);   break;
    case ITEM_UINT16: *(volatile uint16_t *)it->var = (uint16_t)round_f(v);  break;
    case ITEM_FLOAT:  *(volatile float    *)it->var = v;                     break;
    case ITEM_BOOL:   *(volatile uint8_t  *)it->var = (v != 0.0f) ? 1u : 0u; break;
    default: break;
    }
}

static void build_label(char *dst, const char *name)
{
    uint8_t i;
    for (i = 0; i < MENU_COLS; i++) dst[i] = ' ';
    dst[MENU_COLS] = '\0';
    if (name) for (i = 0; i < MENU_COLS && name[i]; i++) dst[i] = name[i];
}

static void draw_title(const char *txt)
{
    char t[MENU_COLS + 1];
    uint8_t i, j;
    for (i = 0; i < MENU_COLS; i++) t[i] = ' ';
    t[MENU_COLS] = '\0';
    t[0] = '='; t[1] = ' '; j = 2;
    for (i = 0; txt && txt[i] && j < MENU_COLS - 2; i++) t[j++] = txt[i];
    if (j < MENU_COLS - 1) { t[j++] = ' '; t[j++] = '='; }
    menu_port_draw_text(0, 0, t, MENU_STYLE_TITLE);
}

static void draw_title_edit(const char *name)
{
    char t[MENU_COLS + 1];
    const char *pfx = "[E] ";
    uint8_t i, j = 0;
    for (i = 0; i < MENU_COLS; i++) t[i] = ' ';
    t[MENU_COLS] = '\0';
    for (i = 0; pfx[i] && j < MENU_COLS; i++) t[j++] = pfx[i];
    for (i = 0; name && name[i] && j < MENU_COLS; i++) t[j++] = name[i];
    menu_port_draw_text(0, 0, t, MENU_STYLE_EDIT);
}

static void draw_value(const menu_item_t *it, uint8_t row, float v, menu_style_e st)
{
    switch (it->type)
    {
    case ITEM_FLOAT:  menu_port_draw_float(VALUE_COL, row, v, 4, 2, st);                       break;
    case ITEM_INT16:  menu_port_draw_int (VALUE_COL, row, (int32_t)round_f(v), VALUE_WIDTH, st); break;
    case ITEM_UINT16: menu_port_draw_uint(VALUE_COL, row, (uint32_t)round_f(v), VALUE_WIDTH, st);break;
    case ITEM_BOOL:   menu_port_draw_text(VALUE_COL, row, (v != 0.0f) ? "ON " : "OFF", st);    break;
    default: break;
    }
}

static void draw_item_row(uint8_t index, uint8_t screen_row, bool selected, bool editing)
{
    const menu_item_t *it = &menu_items[index];
    char label[MENU_COLS + 1];
    menu_style_e st = editing ? MENU_STYLE_EDIT : (selected ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
    build_label(label, it->name);
    if (selected || editing) { label[0] = '>'; label[1] = ' '; }
    menu_port_draw_text(0, screen_row, label, st);
    if (it->type != ITEM_ACTION) draw_value(it, screen_row, editing ? s_edit_val : item_get(it), st);
}

static void draw_list_full(void)
{
    char blank[MENU_COLS + 1];
    uint8_t r;
    menu_port_clear();
    draw_title("TUNING");
    build_label(blank, "");
    for (r = 0; r < CONTENT_ROWS; r++)
    {
        uint8_t i = (uint8_t)(s_top + r);
        if (i < menu_item_count)
            draw_item_row(i, (uint8_t)(r + 1), (i == s_cursor), (s_nav == NAV_EDIT && i == s_cursor));
        else
            menu_port_draw_text(0, (uint8_t)(r + 1), blank, MENU_STYLE_NORMAL);
    }
}

static void redraw_visible_values(void)
{
    uint8_t r;
    for (r = 0; r < CONTENT_ROWS; r++)
    {
        uint8_t i = (uint8_t)(s_top + r);
        if (i >= menu_item_count) break;
        if (menu_items[i].type != ITEM_ACTION)
            draw_value(&menu_items[i], (uint8_t)(r + 1), item_get(&menu_items[i]),
                       (i == s_cursor) ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
    }
}

static void list_move(int8_t dir)
{
    uint8_t old = s_cursor, old_top = s_top;
    if (dir < 0) { if (s_cursor == 0) return; s_cursor--; }
    else         { if (s_cursor + 1 >= menu_item_count) return; s_cursor++; }

    if (s_cursor < s_top)                         s_top = s_cursor;
    else if (s_cursor >= s_top + CONTENT_ROWS)    s_top = (uint8_t)(s_cursor - CONTENT_ROWS + 1);

    if (s_top != old_top) { draw_list_full(); return; }
    draw_item_row(old,      (uint8_t)((old - s_top) + 1),      false, false);
    draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true,  false);
}

static void item_enter(void)
{
    const menu_item_t *it = &menu_items[s_cursor];
    if (it->type == ITEM_ACTION) { if (it->action) it->action(); return; }
    if (it->type == ITEM_BOOL)
    {
        item_set(it, (item_get(it) != 0.0f) ? 0.0f : 1.0f);   // 原子 toggle + commit
        draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true, false);
        return;
    }
    s_nav = NAV_EDIT;                                          // 数值 → 进入编辑（工作副本）
    s_edit_item = it;
    s_edit_val  = item_get(it);
    draw_title_edit(it->name);
    draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true, true);
}

static void edit_adjust(int8_t dir, uint8_t repeat)
{
    const menu_item_t *it = s_edit_item;
    float step = it->step * (repeat ? KEY_STEP_MULT : 1.0f);
    s_edit_val += (dir > 0) ? step : -step;
    if (s_edit_val < it->min) s_edit_val = it->min;
    if (s_edit_val > it->max) s_edit_val = it->max;
    draw_value(it, (uint8_t)((s_cursor - s_top) + 1), s_edit_val, MENU_STYLE_EDIT);
}

static void edit_end(bool commit)
{
    if (commit && s_edit_item) item_set(s_edit_item, s_edit_val);   // 单次写回
    s_nav = NAV_LIST;
    draw_title("TUNING");
    draw_item_row(s_cursor, (uint8_t)((s_cursor - s_top) + 1), true, false);
}

static void apply_defaults(void)
{
    uint16_t i;
    for (i = 0; i < menu_item_count; i++)
        if (menu_items[i].type != ITEM_ACTION) item_set(&menu_items[i], menu_items[i].def);
}

void menu_action_defaults(void)
{
    apply_defaults();
    draw_title("Defaults");
    redraw_visible_values();
}

void menu_action_camera(void)
{
    s_align_test_mode = 0;
    s_motor_test_mode = 0;
    s_camera_view = 1;
}

void menu_action_align_test(void)
{
    s_motor_test_mode = 0;
    s_align_test_mode = 1;
    s_camera_view = 1;
    motor_stop();
}

void menu_action_motor_test(void)
{
    s_align_test_mode = 0;
    s_camera_view = 0;
    s_motor_test_mode = 1;
    motor_stop();
    menu_port_clear();
    draw_title("Motor Test");
    menu_port_draw_text(0, 2, "Duty: 20%", MENU_STYLE_NORMAL);
    menu_port_draw_text(0, 4, "BACK: stop", MENU_STYLE_NORMAL);
}

void menu_action_reset(void)
{
    control_reset();
    motor_reset();
    draw_title("Reset");
}

uint8_t menu_camera_view(void)
{
    return s_camera_view;
}

uint8_t menu_align_test_mode(void)
{
    return s_align_test_mode;
}

uint8_t menu_motor_test_mode(void)
{
    return s_motor_test_mode;
}

void menu_init(void)
{
    menu_port_init();
    apply_defaults();
    drive_armed = 0;
    s_nav = NAV_LIST;
    s_cursor = 0;
    s_top = 0;
    draw_list_full();
}

static void menu_handle_key(const menu_key_event_t *ev)
{
    if (s_motor_test_mode)
    {
        if (ev->key == MENU_KEY_BACK)
        {
            s_motor_test_mode = 0;
            motor_stop();
            draw_list_full();
        }
        return;
    }

    if (s_camera_view)
    {
        if (ev->key == MENU_KEY_BACK)
        {
            s_camera_view = 0;
            s_align_test_mode = 0;
            draw_list_full();
        }
        return;
    }

    if (s_nav == NAV_LIST)
    {
        if      (ev->key == MENU_KEY_UP)    list_move(-1);
        else if (ev->key == MENU_KEY_DOWN)  list_move(+1);
        else if (ev->key == MENU_KEY_ENTER) item_enter();
    }
    else /* NAV_EDIT */
    {
        if      (ev->key == MENU_KEY_UP)    edit_adjust(+1, ev->is_repeat);
        else if (ev->key == MENU_KEY_DOWN)  edit_adjust(-1, ev->is_repeat);
        else if (ev->key == MENU_KEY_ENTER) edit_end(true);
        else if (ev->key == MENU_KEY_BACK)  edit_end(false);
    }
}

void menu_task(void)
{
    menu_key_event_t ev;

    menu_port_key_scan();
    while (1)
    {
        menu_port_scan_keys(&ev);
        if (ev.key == MENU_KEY_NONE)
        {
            break;
        }
        menu_handle_key(&ev);
    }
}
