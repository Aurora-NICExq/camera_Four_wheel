/*********************************************************************************************************************
 * menu.h  --  Data-driven parameter-tuning menu : hardware-independent core (public API + config types)
 *
 * This header is 100% hardware independent. It contains NO SeekFree calls. It is included by:
 *   - menu.c         (the engine)
 *   - menu_config.c  (the ONE place items / pages / monitor-fields are defined)
 *
 * Scalar types use <stdint.h>. Note that on the SeekFree TC264 library:
 *      int16_t  == int16 ,  uint16_t == uint16 ,  uint8_t == uint8 ,  uint32_t == uint32
 * so the menu can point straight at your existing volatile globals with zero changes to control code.
 ********************************************************************************************************************/
#ifndef _menu_h_
#define _menu_h_

#include <stdint.h>
#include <stdbool.h>

//====================================================================================================================
// Item / page model
//====================================================================================================================
typedef enum
{
    ITEM_INT16,         // pointer -> volatile int16_t
    ITEM_UINT16,        // pointer -> volatile uint16_t
    ITEM_FLOAT,         // pointer -> volatile float
    ITEM_BOOL,          // pointer -> volatile uint8_t (0/1)
    ITEM_ACTION,        // calls action(), no variable
} item_type_e;

// Top-level pages. To add a page: add an entry BEFORE PAGE_NUM and one row in menu_pages[] (menu_config.c).
typedef enum
{
    PAGE_STEER_PID,
    PAGE_SPEED_PID,
    PAGE_SPEED,
    PAGE_SERVO,
    PAGE_MONITOR,
    PAGE_SYSTEM,
    PAGE_NUM,
} menu_page_e;

typedef enum
{
    PAGE_KIND_NORMAL,   // ENTER shows the item list of this page
    PAGE_KIND_MONITOR,  // ENTER shows a live read-only screen (menu_monitor_fields[])
} page_kind_e;

// One row of the item table == one tunable / action. min/max/step/def are floats and are cast per type,
// so a single struct covers int16 / uint16 / float. Short press = 1x step, long-press auto-repeat = 10x step.
typedef struct
{
    const char  *name;                  // display name, keep it short (<= ~14 chars)
    void        *var;                   // &your_global ; NULL for ACTION items
    item_type_e  type;
    float        min;
    float        max;
    float        step;
    float        def;                   // default value (boot fallback + "Restore Def")
    menu_page_e  page;                  // which page this item lives on
    void       (*action)(void);         // ACTION items only, else NULL
} menu_item_t;

typedef struct
{
    const char  *name;
    page_kind_e  kind;
} menu_page_t;

//====================================================================================================================
// Monitor (live read-only) field
//====================================================================================================================
typedef enum { MON_I16, MON_U16, MON_I32, MON_U32 } mon_type_e;

typedef struct
{
    const char *label;
    const void *var;                    // &your_global (read only)
    mon_type_e  type;
} monitor_field_t;

//====================================================================================================================
// One-line config macros (see the README at the top of menu_config.c)
//====================================================================================================================
#define MENU_F32(nm, v, mn, mx, st, df, pg)  { (nm), (void*)&(v), ITEM_FLOAT , (mn), (mx), (st), (df), (pg), 0 }
#define MENU_I16(nm, v, mn, mx, st, df, pg)  { (nm), (void*)&(v), ITEM_INT16 , (mn), (mx), (st), (df), (pg), 0 }
#define MENU_U16(nm, v, mn, mx, st, df, pg)  { (nm), (void*)&(v), ITEM_UINT16, (mn), (mx), (st), (df), (pg), 0 }
#define MENU_BOOL(nm, v, df, pg)             { (nm), (void*)&(v), ITEM_BOOL  , 0, 1, 1, (df), (pg), 0 }
#define MENU_ACTION(nm, fn, pg)              { (nm), 0, ITEM_ACTION, 0, 0, 0, 0, (pg), (fn) }

//====================================================================================================================
// Config tables (defined in menu_config.c -- the ONLY place they are defined)
//====================================================================================================================
extern const menu_page_t     menu_pages[PAGE_NUM];
extern const menu_item_t     menu_items[];
extern const uint16_t        menu_item_count;
extern const monitor_field_t menu_monitor_fields[];
extern const uint16_t        menu_monitor_count;

//====================================================================================================================
// Flash record identity.  BUMP THE VERSION whenever you add / remove / reorder a *saved* (non-action) item,
// so an old, incompatible flash image is rejected and defaults are used instead.
//====================================================================================================================
#define MENU_FLASH_MAGIC     (0x4D4E5531u)   /* 'M''N''U''1' */
#define MENU_FLASH_VERSION   (1u)

//====================================================================================================================
// Engine API  (all non-blocking)
//====================================================================================================================
void menu_init(void);       // call once on CPU0 after clock_init()/debug_init(); loads flash or applies defaults
void menu_task(void);       // call every CPU0 main-loop iteration; no delays, no busy-waits

// These may be used directly as ACTION functions in menu_config.c (or wrap them in your own).
void menu_action_save(void);        // serialize live values -> flash  (STALLS the CPU; only call when stopped)
void menu_action_load(void);        // flash -> live values (if the record is valid)
void menu_action_defaults(void);    // live values <- table defaults

#endif
