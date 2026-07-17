/*********************************************************************************************************************
 * menu.c  --  Data-driven parameter-tuning menu : hardware-independent engine.
 *
 * Responsibilities:
 *   - navigation state machine  (page list -> item list -> edit ; plus full-screen Monitor mode)
 *   - cursor + scrolling, with PARTIAL dirty-line redraws only (never clears the screen per frame)
 *   - numeric edit with a WORKING COPY : the live volatile global is written only when editing finishes,
 *     so the control loop on CPU1 never observes a half-adjusted value
 *   - table-driven flash save / load / defaults (magic + version + count + checksum)
 *
 * Contains NO SeekFree calls -- every hardware action goes through menu_port_*().
 *
 * OLED adaptation: on a 240x320 screen with 6x8 font (40 cols x 40 rows),
 *   - the cursor is indicated by a "> " prefix on selected rows
 *   - the title bar is decorated as "== NAME =="
 *   - edit mode title is "[E] name"
 ********************************************************************************************************************/
#include "menu.h"
#include "menu_port.h"
#include <string.h>

//====================================================================================================================
// Layout  (240x320, 6x8 font → 40 cols × 40 rows)
//====================================================================================================================
#define CONTENT_ROWS      (MENU_ROWS - 1)       // row 0 is the title bar
#define VALUE_COL         (20)                  // value field starts at column 20
#define VALUE_WIDTH       (MENU_COLS - VALUE_COL)  // 20
#define MENU_MONITOR_MAX  (32)                   // max monitor fields the shadow buffer supports
#define MENU_FLASH_MAX_WORDS (64)                // magic+version+count + params + checksum must fit here
#define KEY_STEP_MULT     (10.0f)                // long-press auto-repeat step multiplier

//====================================================================================================================
// State
//====================================================================================================================
typedef enum { NAV_PAGELIST, NAV_ITEMLIST, NAV_EDIT, NAV_MONITOR } nav_state_e;

static nav_state_e         s_nav;
static uint8_t             s_page_cursor;       // pagelist selection
static uint8_t             s_page_top;          // pagelist scroll window top
static menu_page_e         s_cur_page;          // currently entered page
static uint8_t             s_item_cursor;       // itemlist selection (index within page)
static uint8_t             s_item_top;          // itemlist scroll window top
static float               s_edit_val;          // working copy while editing (committed on ENTER)
static const menu_item_t  *s_edit_item;

static uint32_t            s_last_monitor_ms;

static int32_t             s_mon_shadow[MENU_MONITOR_MAX];
static bool                s_mon_shadow_valid;

static uint32_t            s_flash_buf[MENU_FLASH_MAX_WORDS];

//====================================================================================================================
// Forward declarations
//====================================================================================================================
static uint8_t              page_count(menu_page_e pg);
static const menu_item_t   *page_item(menu_page_e pg, uint8_t n);
static float                item_get(const menu_item_t *it);
static void                 item_set(const menu_item_t *it, float v);
static int32_t              round_f(float v);

static void  build_label(char *dst, const char *name);
static void  draw_title(const char *txt);
static void  draw_title_edit(const char *name);
static void  draw_value_f(const menu_item_t *it, uint8_t row, float v, menu_style_e style);
static void  draw_item_row(uint8_t list_index, uint8_t screen_row, bool selected, bool editing);
static void  draw_itemlist_full(void);
static void  redraw_visible_values(void);
static void  draw_page_row(uint8_t page_index, bool selected);
static void  draw_pagelist_full(void);
static void  draw_monitor_full(void);
static void  draw_monitor_values(bool force);

static void  pagelist_move(int8_t dir);
static void  itemlist_move(int8_t dir);
static void  enter_page(menu_page_e pg);
static void  item_enter(void);
static void  edit_adjust(int8_t dir, uint8_t repeat);
static void  edit_commit(void);
static void  edit_cancel(void);

static void  apply_defaults(void);
static bool  load_from_flash(void);

//====================================================================================================================
// Table helpers
//====================================================================================================================
static uint8_t page_count(menu_page_e pg)
{
    uint8_t c = 0;
    uint16_t i;
    for (i = 0; i < menu_item_count; i++)
        if (menu_items[i].page == pg) c++;
    return c;
}

static const menu_item_t *page_item(menu_page_e pg, uint8_t n)
{
    uint16_t i;
    for (i = 0; i < menu_item_count; i++)
    {
        if (menu_items[i].page == pg)
        {
            if (n == 0) return &menu_items[i];
            n--;
        }
    }
    return 0;
}

static int32_t round_f(float v)
{
    return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

// Read a live global as a float (used for display / edit start value).
static float item_get(const menu_item_t *it)
{
    switch (it->type)
    {
        case ITEM_INT16:  return (float)(*(volatile int16_t  *)it->var);
        case ITEM_UINT16: return (float)(*(volatile uint16_t *)it->var);
        case ITEM_FLOAT:  return         *(volatile float    *)it->var;
        case ITEM_BOOL:   return (*(volatile uint8_t *)it->var) ? 1.0f : 0.0f;
        default:          return 0.0f;
    }
}

// Commit a value to the live global. Each store is a single aligned 16/32-bit write, so the control
// context only ever sees the old value or this new value -- never an intermediate.
static void item_set(const menu_item_t *it, float v)
{
    if (v < it->min) v = it->min;
    if (v > it->max) v = it->max;
    switch (it->type)
    {
        case ITEM_INT16:  *(volatile int16_t  *)it->var = (int16_t)round_f(v);          break;
        case ITEM_UINT16: *(volatile uint16_t *)it->var = (uint16_t)round_f(v);         break;
        case ITEM_FLOAT:  *(volatile float    *)it->var = v;                            break;
        case ITEM_BOOL:   *(volatile uint8_t  *)it->var = (v != 0.0f) ? 1u : 0u;        break;
        default: break;
    }
}

//====================================================================================================================
// Text formatting helpers  (no sprintf : numbers go through the oled show functions in the port)
//====================================================================================================================

// Build a full-width (MENU_COLS) space-padded label so the highlight bar spans the whole row.
static void build_label(char *dst, const char *name)
{
    uint8_t i;
    for (i = 0; i < MENU_COLS; i++) dst[i] = ' ';
    dst[MENU_COLS] = '\0';
    if (name)
        for (i = 0; i < MENU_COLS && name[i]; i++) dst[i] = name[i];
}

// OLED title: "== NAME =="
static void draw_title(const char *txt)
{
    char t[MENU_COLS + 1];
    uint8_t i, j = 0;
    for (i = 0; i < MENU_COLS; i++) t[i] = ' ';
    t[0] = '='; t[1] = ' ';
    j = 2;
    for (i = 0; txt && txt[i] && j < MENU_COLS - 2; i++) t[j++] = txt[i];
    if (j < MENU_COLS - 1) { t[j++] = ' '; t[j++] = '='; }
    t[MENU_COLS] = '\0';
    menu_port_draw_text(0, 0, t, MENU_STYLE_TITLE);
}

// OLED edit title: "[E] name"
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

// Draw the value field of an item at `row`, formatted per type, using `v` (the live value or the edit copy).
static void draw_value_f(const menu_item_t *it, uint8_t row, float v, menu_style_e style)
{
    switch (it->type)
    {
        case ITEM_FLOAT:  menu_port_draw_float(VALUE_COL, row, v, 4, 2, style);                        break;
        case ITEM_INT16:  menu_port_draw_int  (VALUE_COL, row, (int32_t)round_f(v), VALUE_WIDTH, style); break;
        case ITEM_UINT16: menu_port_draw_uint (VALUE_COL, row, (uint32_t)round_f(v), VALUE_WIDTH, style); break;
        case ITEM_BOOL:   menu_port_draw_text (VALUE_COL, row, (v != 0.0f) ? "ON " : "OFF", style);      break;
        default: break;
    }
}

//====================================================================================================================
// Item-list rendering
//====================================================================================================================
static void draw_item_row(uint8_t list_index, uint8_t screen_row, bool selected, bool editing)
{
    const menu_item_t *it = page_item(s_cur_page, list_index);
    char label[MENU_COLS + 1];
    menu_style_e st = editing ? MENU_STYLE_EDIT : (selected ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
    if (!it) return;

    build_label(label, it->name);

    // OLED cursor: prefix "> " on selected / editing rows
    if (selected || editing)
    {
        label[0] = '>';
        label[1] = ' ';
    }

    menu_port_draw_text(0, screen_row, label, st);
    if (it->type != ITEM_ACTION)
        draw_value_f(it, screen_row, editing ? s_edit_val : item_get(it), st);
}

static void draw_itemlist_full(void)
{
    uint8_t count = page_count(s_cur_page);
    uint8_t r;
    char blank[MENU_COLS + 1];
    menu_port_clear();
    draw_title(menu_pages[s_cur_page].name);
    build_label(blank, "");
    for (r = 0; r < CONTENT_ROWS; r++)
    {
        uint8_t li = (uint8_t)(s_item_top + r);
        if (li < count)
            draw_item_row(li, (uint8_t)(r + 1), (li == s_item_cursor), (s_nav == NAV_EDIT && li == s_item_cursor));
        else
            menu_port_draw_text(0, (uint8_t)(r + 1), blank, MENU_STYLE_NORMAL);
    }
}

// Refresh only the value fields of the visible rows (after Load / Restore-Defaults changed live values).
static void redraw_visible_values(void)
{
    uint8_t count = page_count(s_cur_page);
    uint8_t r;
    for (r = 0; r < CONTENT_ROWS; r++)
    {
        uint8_t li = (uint8_t)(s_item_top + r);
        const menu_item_t *it;
        if (li >= count) break;
        it = page_item(s_cur_page, li);
        if (it && it->type != ITEM_ACTION)
            draw_value_f(it, (uint8_t)(r + 1), item_get(it), (li == s_item_cursor) ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
    }
}

//====================================================================================================================
// Page-list rendering
//====================================================================================================================
static void draw_page_row(uint8_t page_index, bool selected)
{
    char label[MENU_COLS + 1];
    build_label(label, menu_pages[page_index].name);
    if (selected) { label[0] = '>'; label[1] = ' '; }
    menu_port_draw_text(0, (uint8_t)((page_index - s_page_top) + 1), label,
                        selected ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
}

static void draw_pagelist_full(void)
{
    uint8_t r;
    char label[MENU_COLS + 1];
    menu_port_clear();
    draw_title("MAIN MENU");
    for (r = 0; r < CONTENT_ROWS; r++)
    {
        uint8_t pi = (uint8_t)(s_page_top + r);
        if (pi < PAGE_NUM)
        {
            build_label(label, menu_pages[pi].name);
            if (pi == s_page_cursor) { label[0] = '>'; label[1] = ' '; }
            menu_port_draw_text(0, (uint8_t)(r + 1), label,
                (pi == s_page_cursor) ? MENU_STYLE_SELECTED : MENU_STYLE_NORMAL);
        }
        else
        {
            build_label(label, "");
            menu_port_draw_text(0, (uint8_t)(r + 1), label, MENU_STYLE_NORMAL);
        }
    }
}

//====================================================================================================================
// Monitor rendering (live, ~10 Hz, only changed values are redrawn)
//====================================================================================================================
static int32_t mon_read(const monitor_field_t *f)
{
    switch (f->type)
    {
        case MON_I16: return (int32_t)(*(volatile int16_t  *)f->var);
        case MON_U16: return (int32_t)(*(volatile uint16_t *)f->var);
        case MON_I32: return           *(volatile int32_t  *)f->var;
        case MON_U32: return (int32_t)(*(volatile uint32_t *)f->var);   // reinterpreted bits; compare-only
        default:      return 0;
    }
}

static void draw_monitor_values(bool force)
{
    uint16_t i;
    for (i = 0; i < menu_monitor_count && i < CONTENT_ROWS && i < MENU_MONITOR_MAX; i++)
    {
        int32_t v = mon_read(&menu_monitor_fields[i]);
        if (force || !s_mon_shadow_valid || s_mon_shadow[i] != v)
        {
            if (menu_monitor_fields[i].type == MON_U32)
                menu_port_draw_uint(VALUE_COL, (uint8_t)(i + 1), (uint32_t)v, VALUE_WIDTH, MENU_STYLE_NORMAL);
            else
                menu_port_draw_int(VALUE_COL, (uint8_t)(i + 1), v, VALUE_WIDTH, MENU_STYLE_NORMAL);
            s_mon_shadow[i] = v;
        }
    }
    s_mon_shadow_valid = true;
}

static void draw_monitor_full(void)
{
    uint16_t i;
    char label[MENU_COLS + 1];
    menu_port_clear();
    draw_title("Monitor");
    for (i = 0; i < menu_monitor_count && i < CONTENT_ROWS && i < MENU_MONITOR_MAX; i++)
    {
        build_label(label, menu_monitor_fields[i].label);
        menu_port_draw_text(0, (uint8_t)(i + 1), label, MENU_STYLE_NORMAL);
    }
    s_mon_shadow_valid = false;         // force every value to draw once
    draw_monitor_values(true);
}

//====================================================================================================================
// Navigation actions
//====================================================================================================================
static void pagelist_move(int8_t dir)
{
    uint8_t old = s_page_cursor;
    uint8_t old_top = s_page_top;
    if (dir < 0) { if (s_page_cursor == 0) return; s_page_cursor--; }
    else         { if (s_page_cursor + 1 >= PAGE_NUM) return; s_page_cursor++; }

    if (s_page_cursor < s_page_top)                         s_page_top = s_page_cursor;
    else if (s_page_cursor >= s_page_top + CONTENT_ROWS)    s_page_top = (uint8_t)(s_page_cursor - CONTENT_ROWS + 1);

    if (s_page_top != old_top) { draw_pagelist_full(); return; }
    draw_page_row(old, false);
    draw_page_row(s_page_cursor, true);
}

static void itemlist_move(int8_t dir)
{
    uint8_t count = page_count(s_cur_page);
    uint8_t old = s_item_cursor;
    uint8_t old_top = s_item_top;
    if (count == 0) return;
    if (dir < 0) { if (s_item_cursor == 0) return; s_item_cursor--; }
    else         { if (s_item_cursor + 1 >= count) return; s_item_cursor++; }

    if (s_item_cursor < s_item_top)                         s_item_top = s_item_cursor;
    else if (s_item_cursor >= s_item_top + CONTENT_ROWS)    s_item_top = (uint8_t)(s_item_cursor - CONTENT_ROWS + 1);

    if (s_item_top != old_top) { draw_itemlist_full(); return; }
    draw_item_row(old,           (uint8_t)((old - s_item_top) + 1),           false, false);
    draw_item_row(s_item_cursor, (uint8_t)((s_item_cursor - s_item_top) + 1), true,  false);
}

static void enter_page(menu_page_e pg)
{
    s_cur_page   = pg;
    s_item_cursor = 0;
    s_item_top    = 0;
    if (menu_pages[pg].kind == PAGE_KIND_MONITOR)
    {
        s_nav = NAV_MONITOR;
        s_last_monitor_ms = menu_port_millis();
        draw_monitor_full();
    }
    else
    {
        s_nav = NAV_ITEMLIST;
        draw_itemlist_full();
    }
}

static void item_enter(void)
{
    const menu_item_t *it = page_item(s_cur_page, s_item_cursor);
    if (!it) return;

    if (it->type == ITEM_ACTION)
    {
        if (it->action) it->action();               // action may change s_nav (e.g. Show Image)
        return;
    }
    if (it->type == ITEM_BOOL)
    {
        item_set(it, (item_get(it) != 0.0f) ? 0.0f : 1.0f);     // atomic toggle+commit
        draw_item_row(s_item_cursor, (uint8_t)((s_item_cursor - s_item_top) + 1), true, false);
        return;
    }
    // numeric -> enter edit mode on a working copy
    s_nav      = NAV_EDIT;
    s_edit_item = it;
    s_edit_val  = item_get(it);
    draw_title_edit(it->name);
    draw_item_row(s_item_cursor, (uint8_t)((s_item_cursor - s_item_top) + 1), true, true);
}

static void edit_adjust(int8_t dir, uint8_t repeat)
{
    const menu_item_t *it = s_edit_item;
    float step;
    if (!it) return;
    step = it->step * (repeat ? KEY_STEP_MULT : 1.0f);
    s_edit_val += (dir > 0) ? step : -step;
    if (s_edit_val < it->min) s_edit_val = it->min;
    if (s_edit_val > it->max) s_edit_val = it->max;
    draw_value_f(it, (uint8_t)((s_item_cursor - s_item_top) + 1), s_edit_val, MENU_STYLE_EDIT);
}

static void edit_commit(void)
{
    const menu_item_t *it = s_edit_item;
    if (it) item_set(it, s_edit_val);               // <-- the single commit; control now sees the final value
    s_nav = NAV_ITEMLIST;
    draw_title(menu_pages[s_cur_page].name);
    draw_item_row(s_item_cursor, (uint8_t)((s_item_cursor - s_item_top) + 1), true, false);
}

static void edit_cancel(void)
{
    s_nav = NAV_ITEMLIST;
    draw_title(menu_pages[s_cur_page].name);
    draw_item_row(s_item_cursor, (uint8_t)((s_item_cursor - s_item_top) + 1), true, false);  // shows unchanged live value
}


//====================================================================================================================
// Flash save / load / defaults  (table-driven -> adding a param needs no change here)
//====================================================================================================================
static uint16_t savable_count(void)
{
    uint16_t i, c = 0;
    for (i = 0; i < menu_item_count; i++)
        if (menu_items[i].type != ITEM_ACTION) c++;
    return c;
}

static uint32_t float_bits(float f) { union { float f; uint32_t u; } x; x.f = f; return x.u; }
static float    bits_float(uint32_t u) { union { float f; uint32_t u; } x; x.u = u; return x.f; }

static uint32_t item_to_word(const menu_item_t *it)
{
    switch (it->type)
    {
        case ITEM_FLOAT:  return float_bits(*(volatile float *)it->var);
        case ITEM_INT16:  return (uint32_t)(int32_t)(*(volatile int16_t  *)it->var);
        case ITEM_UINT16: return (uint32_t)(*(volatile uint16_t *)it->var);
        case ITEM_BOOL:   return (*(volatile uint8_t *)it->var) ? 1u : 0u;
        default:          return 0u;
    }
}

static void word_to_item(const menu_item_t *it, uint32_t w)
{
    switch (it->type)
    {
        case ITEM_FLOAT:  *(volatile float    *)it->var = bits_float(w);                break;
        case ITEM_INT16:  *(volatile int16_t  *)it->var = (int16_t)(int32_t)w;          break;
        case ITEM_UINT16: *(volatile uint16_t *)it->var = (uint16_t)w;                  break;
        case ITEM_BOOL:   *(volatile uint8_t  *)it->var = w ? 1u : 0u;                  break;
        default: break;
    }
}

static void apply_defaults(void)
{
    uint16_t i;
    for (i = 0; i < menu_item_count; i++)
        if (menu_items[i].type != ITEM_ACTION)
            item_set(&menu_items[i], menu_items[i].def);
}

static bool load_from_flash(void)
{
    uint16_t n = savable_count();
    uint16_t total = (uint16_t)(n + 4);         // magic + version + count + params + checksum
    uint16_t i, k;
    uint32_t sum = 0;
    if (total > MENU_FLASH_MAX_WORDS) return false;

    menu_port_flash_read(s_flash_buf, total);
    if (s_flash_buf[0] != MENU_FLASH_MAGIC)   return false;
    if (s_flash_buf[1] != MENU_FLASH_VERSION) return false;
    if (s_flash_buf[2] != n)                  return false;
    for (i = 0; i < (uint16_t)(total - 1); i++) sum += s_flash_buf[i];
    if (sum != s_flash_buf[total - 1])        return false;

    k = 3;
    for (i = 0; i < menu_item_count; i++)
        if (menu_items[i].type != ITEM_ACTION)
            word_to_item(&menu_items[i], s_flash_buf[k++]);
    return true;
}

void menu_action_save(void)
{
    uint16_t n = savable_count();
    uint16_t total = (uint16_t)(n + 4);
    uint16_t i, k;
    uint32_t sum = 0;
    if (total > MENU_FLASH_MAX_WORDS) { draw_title("Too Many"); return; }

    s_flash_buf[0] = MENU_FLASH_MAGIC;
    s_flash_buf[1] = MENU_FLASH_VERSION;
    s_flash_buf[2] = n;
    k = 3;
    for (i = 0; i < menu_item_count; i++)
        if (menu_items[i].type != ITEM_ACTION)
            s_flash_buf[k++] = item_to_word(&menu_items[i]);
    for (i = 0; i < k; i++) sum += s_flash_buf[i];
    s_flash_buf[k++] = sum;

    draw_title(menu_port_flash_write(s_flash_buf, k) ? "Saved" : "Save ERR");
}

void menu_action_load(void)
{
    draw_title(load_from_flash() ? "Loaded" : "No Data");
    if (s_nav == NAV_ITEMLIST) redraw_visible_values();
}

void menu_action_defaults(void)
{
    apply_defaults();
    draw_title("Defaults");
    if (s_nav == NAV_ITEMLIST) redraw_visible_values();
}


//====================================================================================================================
// Public entry points
//====================================================================================================================
void menu_init(void)
{
    menu_port_init();
    if (!load_from_flash()) apply_defaults();    // valid record -> use it; else fall back to table defaults

    s_nav         = NAV_PAGELIST;
    s_page_cursor = 0;
    s_page_top    = 0;
    draw_pagelist_full();
}

void menu_task(void)
{
    menu_key_event_t ev;
    menu_port_scan_keys(&ev);

    switch (s_nav)
    {
        case NAV_PAGELIST:
            if      (ev.key == MENU_KEY_UP)    pagelist_move(-1);
            else if (ev.key == MENU_KEY_DOWN)  pagelist_move(+1);
            else if (ev.key == MENU_KEY_ENTER) enter_page((menu_page_e)s_page_cursor);
            /* BACK at top level: nothing */
            break;

        case NAV_ITEMLIST:
            if      (ev.key == MENU_KEY_UP)    itemlist_move(-1);
            else if (ev.key == MENU_KEY_DOWN)  itemlist_move(+1);
            else if (ev.key == MENU_KEY_ENTER) item_enter();
            else if (ev.key == MENU_KEY_BACK)  { s_nav = NAV_PAGELIST; draw_pagelist_full(); }
            break;

        case NAV_EDIT:
            if      (ev.key == MENU_KEY_UP)    edit_adjust(+1, ev.is_repeat);
            else if (ev.key == MENU_KEY_DOWN)  edit_adjust(-1, ev.is_repeat);
            else if (ev.key == MENU_KEY_ENTER) edit_commit();
            else if (ev.key == MENU_KEY_BACK)  edit_cancel();
            break;

        case NAV_MONITOR:
            if (ev.key == MENU_KEY_BACK) { s_nav = NAV_PAGELIST; draw_pagelist_full(); }
            else
            {
                uint32_t now = menu_port_millis();
                if ((uint32_t)(now - s_last_monitor_ms) >= 100u) { s_last_monitor_ms = now; draw_monitor_values(false); }
            }
            break;
    }
}
