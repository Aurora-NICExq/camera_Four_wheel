/*********************************************************************************************************************
 * 模块：hybrid_track.h — 最长白列锚定 + 八邻域双边跟踪
 *
 * 本模块只负责逐行几何（left/right/mid/width/lost/valid_rows），阈值计算、误差、曲率和元素检测
 * 仍由 image.c 负责。这样 track_info_t 的对外契约不变，fsm/control/display 无需跟着修改。
 ********************************************************************************************************************/
#ifndef HYBRID_TRACK_H
#define HYBRID_TRACK_H

#include <stdint.h>
#include "../image.h"

/* 可选诊断量：传 NULL 时没有额外输出。longest_rows 与 traced_rows 的差值就是融合算法增加的前瞻。 */
typedef struct
{
    uint8_t longest_rows;       /* 固定最长白列从底向上的连续白行数                         */
    uint8_t traced_rows;        /* 八邻域/预测跟踪后的有效行数，等于 out->valid_rows          */
    uint8_t bridged_rows;       /* 位于有效区间内、靠预测跨过的全黑/断裂行数                 */
    uint8_t left_reacquires;    /* 左边离开严格八邻域后，经小窗口重新捕获的次数               */
    uint8_t right_reacquires;   /* 右边离开严格八邻域后，经小窗口重新捕获的次数               */
} hybrid_track_diag_t;

/*
 * img[0] 是相机原始顶行；输出 row=0 仍表示图像最底行。
 * threshold 与 image.c 使用同一个阈值，避免两套二值化判据互相打架。
 */
void hybrid_track_extract(const uint8_t img[IMG_H][IMG_W],
                          uint8_t threshold,
                          track_info_t *out,
                          hybrid_track_diag_t *diag);

#endif /* HYBRID_TRACK_H */
