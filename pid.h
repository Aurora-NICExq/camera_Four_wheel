#ifndef _pid_h__
#define _pid_h__

#include <stdint.h>

typedef struct {
  float Kp;
  float Kd;

  float current_error;
  float last_error;

  float output;
  float output_limit;
} PositionalPD;

typedef struct {
  float Kp;
  float Ki;
  float Kd;

  float current_error;
  float last_error;
  float previous_error; // e(k-2),增量式pid

  float integral;
  float integral_limit;

  float output;
  float output_limit;
} incre_pid;

// api初始化

void incre_pid_init(incre_pid *pid, float kp, float ki, float kd,
                    float integral_limit, float output_limit);

void PositionalPD_Init(PositionalPD *pd, float kp, float kd,
                       float output_limit);

// 计算接口函数

float incre_pid_compute(incre_pid *pid, float setpoint, flaot measured);

float PositionalPD(PositionalPD *pd, float error);

// 运行调参
void incre_pid_set_tuning(incre_pid *pid, float kp, float ki, float kd);
void PositionalPD_set_tuning(PositionalPD *pid, floatkp, float kd);

// 复位

void incre_pid_reset(incre_pid *pid);
void PositionalPD_reset(PositionalPD *pd);

#endif
