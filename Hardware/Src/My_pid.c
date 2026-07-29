#include "My_pid.h"

/**
  * @brief 计算浮点绝对值。
  * @param value 输入值
  * @retval 输入值的绝对值
  */
static float My_pid_abs_My(float value)
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
static float My_pid_limit_My(float value, float limit)
{
  float abs_limit; /* 统一转成正数后的限幅阈值。 */

  if (limit <= 0.0f)
  {
    return value;
  }

  abs_limit = My_pid_abs_My(limit);
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

void My_pid_init_My(volatile My_pid_t *pid,
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
  My_pid_reset_My(pid);
}

void My_pid_reset_My(volatile My_pid_t *pid)
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

float My_pid_calc_position_My(volatile My_pid_t *pid, float target, float actual)
{
  if (pid == 0)
  {
    return 0.0f;
  }

  pid->error = target - actual;
  pid->integral += pid->error;
  pid->integral = My_pid_limit_My(pid->integral, pid->integral_limit);
  pid->output = pid->kp * pid->error
              + pid->ki * pid->integral
              + pid->kd * (pid->error - pid->last_error);
  pid->output = My_pid_limit_My(pid->output, pid->output_limit);
  pid->prev_error = pid->last_error;
  pid->last_error = pid->error;

  return pid->output;
}

float My_pid_calc_angle_My(volatile My_pid_t *pid, float target, float actual)
{
  float error; /* 折算到 -180 到 180 度范围内的角度误差。 */

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
  pid->integral = My_pid_limit_My(pid->integral, pid->integral_limit);
  pid->output = pid->kp * pid->error
              + pid->ki * pid->integral
              + pid->kd * (pid->error - pid->last_error);
  pid->output = My_pid_limit_My(pid->output, pid->output_limit);
  pid->prev_error = pid->last_error;
  pid->last_error = pid->error;

  return pid->output;
}

float My_pid_calc_incremental_My(volatile My_pid_t *pid,
                                       float target,
                                       float actual,
                                       float target_limit)
{
  if (pid == 0)
  {
    return 0.0f;
  }

  target = My_pid_limit_My(target, target_limit);
  pid->error = target - actual;
  pid->output += pid->kp * (pid->error - pid->last_error)
               + pid->ki * pid->error
               + pid->kd * (pid->error - 2.0f * pid->last_error + pid->prev_error);
  pid->output = My_pid_limit_My(pid->output, pid->output_limit);
  pid->prev_error = pid->last_error;
  pid->last_error = pid->error;

  return pid->output;
}
