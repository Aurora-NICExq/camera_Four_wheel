#include <pid.h>

// pid初始化

void PositionalPD_Init(PositionalPD *pd, float kp, float kd,
                       float output_limit) {
  pd->Kp = kp;
  pd->Kd = kd;
  pd->current_error = 0.0f;
  pd->last_error = 0.0f;
  pd->output = 0.0f;
  pd->output_limit = output_limit;
}

void incre_pid_init(incre_pid *pid, float kp, float ki, float kd,
                    float integral_limit, float output_limit) {
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->current_error = 0.0f;
  pid->last_error = 0.0f;
  pid->previous_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
  pid->output_limit = output_limit;
  pid->integral_limit = integral_limit;
}
