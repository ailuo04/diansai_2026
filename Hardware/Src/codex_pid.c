#include "codex_pid.h"

/**
  * @brief 计算浮点绝对值。
  * @param value 输入值
  * @retval 输入值的绝对值
  */
static float codex_pid_abs_codex(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }

  return value;
}

/**
  * @brief 对浮点值做正负对称限幅。
  * @param value 输入值
  * @param limit 限幅绝对值；小于等于 0 时不限制
  * @retval 限幅后的值
  */
static float codex_pid_limit_codex(float value, float limit)
{
  float abs_limit;

  if (limit <= 0.0f)
  {
    return value;
  }

  abs_limit = codex_pid_abs_codex(limit);
  if (value > abs_limit)
  {
    return abs_limit;
  }
  if (value < -abs_limit)
  {
    return -abs_limit;
  }

  return value;
}

void codex_pid_init_codex(volatile codex_pid_t *pid,
                          float kp,
                          float ki,
                          float kd,
                          float integral_limit,
                          float output_limit)
{
  if (pid == 0)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
  codex_pid_reset_codex(pid);
}

void codex_pid_reset_codex(volatile codex_pid_t *pid)
{
  if (pid == 0)
  {
    return;
  }

  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->prev_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

float codex_pid_calc_position_codex(volatile codex_pid_t *pid, float target, float actual)
{
  if (pid == 0)
  {
    return 0.0f;
  }

  pid->error = target - actual;
  pid->integral += pid->error;
  pid->integral = codex_pid_limit_codex(pid->integral, pid->integral_limit);
  pid->output = pid->kp * pid->error
              + pid->ki * pid->integral
              + pid->kd * (pid->error - pid->last_error);
  pid->output = codex_pid_limit_codex(pid->output, pid->output_limit);
  pid->prev_error = pid->last_error;
  pid->last_error = pid->error;

  return pid->output;
}

float codex_pid_calc_angle_codex(volatile codex_pid_t *pid, float target, float actual)
{
  float error;

  if (pid == 0)
  {
    return 0.0f;
  }

  error = target - actual;
  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }

  pid->error = error;
  pid->integral += pid->error;
  pid->integral = codex_pid_limit_codex(pid->integral, pid->integral_limit);
  pid->output = pid->kp * pid->error
              + pid->ki * pid->integral
              + pid->kd * (pid->error - pid->last_error);
  pid->output = codex_pid_limit_codex(pid->output, pid->output_limit);
  pid->prev_error = pid->last_error;
  pid->last_error = pid->error;

  return pid->output;
}

float codex_pid_calc_incremental_codex(volatile codex_pid_t *pid,
                                       float target,
                                       float actual,
                                       float target_limit)
{
  if (pid == 0)
  {
    return 0.0f;
  }

  target = codex_pid_limit_codex(target, target_limit);
  pid->error = target - actual;
  pid->output += pid->kp * (pid->error - pid->last_error)
               + pid->ki * pid->error
               + pid->kd * (pid->error - 2.0f * pid->last_error + pid->prev_error);
  pid->output = codex_pid_limit_codex(pid->output, pid->output_limit);
  pid->prev_error = pid->last_error;
  pid->last_error = pid->error;

  return pid->output;
}
