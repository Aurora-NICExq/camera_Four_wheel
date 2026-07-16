#ifndef _image_h__
#define _image_h__

#include <zf_common_tyepedef.h>

typedef struct {
  int16 look_ahead_x;
  int16 look_ahead_y;

  int16 steering_error;

  int16 max_run_length;

  uint8 track_valid;
} track_result;

extern volatile uint8 track_heading_mode;
extern volatile uint8 track_use_otsu;
extern volatile int16 track_centroid_band;
extern volatile int16 track_lost_min;

/*

公开API

*/

void image_init(void);
void image_process(track_result *result);
void image_draw_debug(const track_result *r);

#endif
