/*********************************************************************************************************************
 * 模块：image.h — 图像流水线与元素检测器（纯逻辑层）
 *
 * 约束：本头文件及 image.c 不得包含任何 MCU / 逐飞库头文件，只允许 <stdint.h> 与 config.h。
 *
 * 精简版：已移除十字 / 环岛检测器；保留边线跟踪、加权误差、曲率与坡道占位检测。
 ********************************************************************************************************************/
#ifndef IMAGE_H
#define IMAGE_H

#include <stdint.h>
#include "config.h"

typedef struct
{
    /* ---- 逐行几何（仅 [0, valid_rows) 区间有效） ---- */
    uint8_t  left [IMG_H];
    uint8_t  right[IMG_H];
    uint8_t  mid  [IMG_H];
    uint8_t  width[IMG_H];
    uint8_t  left_lost [IMG_H];
    uint8_t  right_lost[IMG_H];

    /* ---- 帧级汇总 ---- */
    uint8_t  valid_rows;
    uint8_t  longest_col;
    int16_t  error;
    int16_t  curvature;
    uint8_t  both_lost_rows;    /* 双边同时丢失的行数（失控保护输入）              */
    uint8_t  threshold;

    /* ---- 元素检测器原始输出（单帧判定，未去抖 —— 去抖只发生在 fsm.c） ---- */
    uint8_t  det_ramp;          /* 1 = 本帧呈坡道特征（占位，默认关闭）           */
    int16_t  det_value;         /* 触发检测器的特征强度，供 FSM 转移轨迹记录      */
} track_info_t;

void image_process(const uint8_t img[IMG_H][IMG_W], uint16_t duty_now, track_info_t *out);

uint8_t image_track_invalid(const track_info_t *ti, uint8_t *severe);

uint8_t detect_ramp(const track_info_t *ti);

#endif /* IMAGE_H */
