/*
 
限定相关区域，防止出现远处无关区域

*/

#include <stdint.h>
#include <zf_common_headfile.h>
#include <image.h>

#define IMG_W (MT9V03X_W) //188
#define IMG_H (MT9V03X_H) //120
#define PIX_WHITE(1)
#define PIX_BLACK(0)

// 可调参数默认值
volatile uint8 track_heading_mode=0;
volatile uint8 track_use_otsu=0;
volatile int16 track_centroid_band=4;
volatile uint8 track_lost_min=10;

//复用参数
extern volatile int16 image_



