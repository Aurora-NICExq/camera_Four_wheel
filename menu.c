/* menu.c */
#include "menu.h"
#include "menu_port.h"
#include "control.h"
#include "image.h"
#include "motor.h"
#include "telemetry.h"

extern volatile float    steer_kp;
volatile uint8_t menu_fine_step = 0;

extern volatile float    steer_kd;
extern volatile float    steer_d_filt_alpha;
extern volatile int16_t  image_threshold;
extern volatile uint8_t  image_cross_fill;
extern volatile uint16_t steer_look_far;

#define CONTENT_ROWS  (MENU_ROWS - 1)          // 第 0 行为标题
#define VALUE_COL     (18)                     // 竖屏 30 列：左侧标签，右侧数值
#define VALUE_WIDTH   (MENU_COLS - VALUE_COL)
#define KEY_STEP_MULT (10.0f)                   // 长按自动重复步长倍率
#define FINE_STEP_DIV  (0.1f)                   // Fine Step 开启时步长倍率
#define FINE_MIN_FLOAT (0.01f)                  // 浮点最小步长(与 2 位小数显示精度对齐)
#define FINE_MIN_INT   (1.0f)                   // 整型最小步长:再小会被取整吃掉,按键像失灵

typedef enum { NAV_LIST, NAV_EDIT, NAV_PRESET } nav_state_e;

#define PRESET_COUNT (3)

typedef struct
{
    const char *name;
    float    kp;
    float    kd;
    float    d_alpha;
    int16_t  threshold;
    uint8_t  cross_fill;
    uint16_t look_far;
    uint16_t duty;
    uint16_t stop_time;
} preset_t;

static const preset_t s_presets[PRESET_COUNT] = {
    { "Low  (tested)",
      PRESET_LOW_KP, PRESET_LOW_KD,
      PRESET_LOW_D_ALPHA,
      PRESET_LOW_THRESHOLD, PRESET_LOW_CROSS_FILL,
      PRESET_LOW_LOOK_FAR, PRESET_LOW_DUTY, PRESET_LOW_STOP_TIME },
    { "Mid  (tested)",
      PRESET_MID_KP, PRESET_MID_KD,
      PRESET_MID_D_ALPHA,
      PRESET_MID_THRESHOLD, PRESET_MID_CROSS_FILL,
      PRESET_MID_LOOK_FAR, PRESET_MID_DUTY, PRESET_MID_STOP_TIME },
    { "High (= Default)",
      PRESET_HIGH_KP, PRESET_HIGH_KD,
      PRESET_HIGH_D_ALPHA,
      PRESET_HIGH_THRESHOLD, PRESET_HIGH_CROSS_FILL,
      PRESET_HIGH_LOOK_FAR, PRESET_HIGH_DUTY, PRESET_HIGH_STOP_TIME },
};

static nav_state_e       s_nav;
static uint8_t           s_camera_view;
static uint8_t           s_align_test_mode;
static uint8_t           s_motor_test_mode;
static uint8_t           s_left_test_mode;
/* Dump Log 结果页。凡是 menu_port_clear() 擦掉列表的页面都必须有自己的
   模式标志 —— menu_handle_key 按标志分发,没标志就落进 NAV_LIST 分支,
   而那里没有 BACK,等于退不出去。 */
static uint8_t           s_dump_view;
static uint8_t           s_preset_cursor;       // 预设子页面选中档位
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
    const char *pfx = menu_fine_step ? "[F] " : "[E] ";
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
    case ITEM_BOOL:
      if (drive_timed_out && v != 0.0f)
        menu_port_draw_text(VALUE_COL, row, "TMO", st);
      else
        menu_port_draw_text(VALUE_COL, row, (v != 0.0f) ? "ON " : "OFF", st);
      break;
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
    float step = it->step;

    if (menu_fine_step)
    {
        float lo = (it->type == ITEM_FLOAT) ? FINE_MIN_FLOAT : FINE_MIN_INT;
        step *= FINE_STEP_DIV;
        if (step < lo) step = lo;
    }
    step *= (repeat ? KEY_STEP_MULT : 1.0f);
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

static void apply_preset(const preset_t *p)
{
    apply_defaults();
    steer_kp           = p->kp;
    steer_kd           = p->kd;
    steer_d_filt_alpha = p->d_alpha;
    image_threshold    = p->threshold;
    image_cross_fill   = p->cross_fill;
    steer_look_far     = p->look_far;
    drive_duty_base    = p->duty;
    drive_stop_time_s  = p->stop_time;
}

static void draw_preset_page(void)
{
    char row[MENU_COLS + 1];
    uint8_t i, j, k;

    menu_port_clear();
    draw_title("PRESET");
    for (i = 0; i < PRESET_COUNT; i++)
    {
        const char *nm = s_presets[i].name;
        for (j = 0; j < MENU_COLS; j++) row[j] = ' ';
        row[MENU_COLS] = '\0';
        j = 2;
        if (i == s_preset_cursor) { row[0] = '>'; row[1] = ' '; }
        for (k = 0; nm[k] && j < MENU_COLS; k++) row[j++] = nm[k];
        menu_port_draw_text(0, (uint8_t)(i + 2), row,
                            (i == s_preset_cursor) ? MENU_STYLE_SELECTED
                                                   : MENU_STYLE_NORMAL);
    }
    menu_port_draw_text(0, (uint8_t)(PRESET_COUNT + 3),
                        "ENTER:apply  BACK:exit", MENU_STYLE_NORMAL);
}

void menu_action_race_preset(void)
{
    s_preset_cursor = 0;
    s_nav = NAV_PRESET;
    draw_preset_page();
}

void menu_action_camera(void)
{
    s_align_test_mode = 0;
    s_motor_test_mode = 0;
    s_left_test_mode  = 0;
    s_camera_view = 1;
}

void menu_action_align_test(void)
{
    s_motor_test_mode = 0;
    s_left_test_mode  = 0;
    s_align_test_mode = 1;
    s_camera_view = 1;
    motor_stop();
}

void menu_action_motor_test(void)
{
    s_align_test_mode = 0;
    s_left_test_mode  = 0;
    s_camera_view = 0;
    s_motor_test_mode = 1;
    motor_stop();
    menu_port_clear();
    draw_title("Motor Test");
    menu_port_draw_text(0, 2, "Duty: 20%", MENU_STYLE_NORMAL);
    menu_port_draw_text(0, 4, "BACK: stop", MENU_STYLE_NORMAL);
}

// 只开 pins.h 里 LEFT 的两路 PWM。转的是哪侧轮 = 软件 LEFT 对应哪侧
void menu_action_left_test(void)
{
    s_align_test_mode = 0;
    s_motor_test_mode = 0;
    s_camera_view = 0;
    s_left_test_mode = 1;
    motor_stop();
    menu_port_clear();
    draw_title("Left Test");
    menu_port_draw_text(0, 2, "Only LEFT PWM", MENU_STYLE_NORMAL);
    menu_port_draw_text(0, 3, "Duty: 20%", MENU_STYLE_NORMAL);
    menu_port_draw_text(0, 5, "See which side", MENU_STYLE_NORMAL);
    menu_port_draw_text(0, 6, "spins", MENU_STYLE_NORMAL);
    menu_port_draw_text(0, 8, "BACK: stop", MENU_STYLE_NORMAL);
}

void menu_action_reset(void)
{
    control_init();
    motor_reset();
    draw_title("Reset");
}

/* 把整圈数据打成 CSV 从调试串口吐出去。阻塞几秒(25 s 的数据约 5 秒),
   期间不刷屏也不扫按键 —— 所以必须先确认车是停的。 */
void menu_action_dump_log(void)
{
    s_align_test_mode = 0;
    s_motor_test_mode = 0;
    s_left_test_mode  = 0;
    s_camera_view = 0;
    s_dump_view = 1;      /* 擦了列表就必须占一个模式,否则 BACK 退不回去 */
    motor_stop();

    menu_port_clear();
    draw_title("Dump Log");

    menu_port_draw_text(0, 7, "BACK to exit", MENU_STYLE_NORMAL);

    if (drive_armed)
    {
        /* Armed 还开着就吐数据,等于一边跑一边卡死主循环几秒 */
        menu_port_draw_text(0, 2, "Armed is ON", MENU_STYLE_NORMAL);
        menu_port_draw_text(0, 3, "Turn it off first", MENU_STYLE_NORMAL);
        return;
    }
    if (telemetry_count() == 0u)
    {
        menu_port_draw_text(0, 2, "No data", MENU_STYLE_NORMAL);
        menu_port_draw_text(0, 4, "Run a lap first", MENU_STYLE_NORMAL);
        return;
    }

    menu_port_draw_text(0, 2, "Frames:", MENU_STYLE_NORMAL);
    menu_port_draw_uint(8, 2, telemetry_count(), 5, MENU_STYLE_NORMAL);
    if (telemetry_overflow())
    {
        /* 缓冲写满被截断。不显示出来的话你会以为拿到的是整圈 */
        menu_port_draw_text(0, 3, "TRUNCATED!", MENU_STYLE_NORMAL);
    }
    menu_port_draw_text(0, 5, "Sending...", MENU_STYLE_NORMAL);

    telemetry_dump();

    menu_port_draw_text(0, 5, "Done      ", MENU_STYLE_NORMAL);
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

uint8_t menu_left_test_mode(void)
{
    return s_left_test_mode;
}

void menu_init(void)
{
    menu_port_init();
    apply_defaults();
    drive_armed = 0;
    drive_timed_out = 0;
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

    if (s_left_test_mode)
    {
        if (ev->key == MENU_KEY_BACK)
        {
            s_left_test_mode = 0;
            motor_stop();
            draw_list_full();
        }
        return;
    }

    if (s_dump_view)
    {
        if (ev->key == MENU_KEY_BACK)
        {
            s_dump_view = 0;
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

    if (s_nav == NAV_PRESET)
    {
        if (ev->key == MENU_KEY_UP)
        {
            if (s_preset_cursor > 0) { s_preset_cursor--; draw_preset_page(); }
        }
        else if (ev->key == MENU_KEY_DOWN)
        {
            if (s_preset_cursor + 1 < PRESET_COUNT) { s_preset_cursor++; draw_preset_page(); }
        }
        else if (ev->key == MENU_KEY_ENTER)
        {
            apply_preset(&s_presets[s_preset_cursor]);
            s_nav = NAV_LIST;
            draw_list_full();
            draw_title(s_presets[s_preset_cursor].name);
        }
        else if (ev->key == MENU_KEY_BACK)
        {
            s_nav = NAV_LIST;
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
