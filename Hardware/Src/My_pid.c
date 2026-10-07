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

/**
  * @brief 使用传感器提供的实际变化速度执行位置式 PID 计算。
  * @param pid PID 参数和历史状态
  * @param target 位置目标
  * @param actual 实际位置
  * @param actual_derivative 传感器返回的实际位置变化速度
  * @param integration_period_s 本周期积分时间；小于等于零时暂停新增积分
  * @retval 完成积分抗饱和和输出限幅后的控制量
  * @details D 项使用负的实际变化速度，避免位置量化差分噪声和目标阶跃引起的微分冲击。
  */
float My_pid_calc_position_with_derivative_My(volatile My_pid_t *pid,
                                               float target,
                                               float actual,
                                               float actual_derivative,
                                               float integration_period_s)
{
  float previous_integral; /* 本周期积分前快照，用于输出饱和时撤销有害积分。 */
  float candidate_integral; /* 按本次有效积分时间计算并限幅后的候选积分。 */
  float integral_output_change; /* 候选积分相对上周期增加的输出量。 */
  float unlimited_output; /* 完成 P、I 和外部速度 D 项叠加但尚未限幅的输出。 */
  float limited_output; /* 按 PID 输出上限约束后的输出。 */

  if (pid == 0)
  {
    return 0.0f;
  }

  pid->error = target - actual;
  previous_integral = pid->integral;
  candidate_integral = previous_integral;
  if (integration_period_s > 0.0f)
  {
    candidate_integral += pid->error * integration_period_s;
    candidate_integral = My_pid_limit_My(candidate_integral, pid->integral_limit);
  }

  unlimited_output = pid->kp * pid->error
                   + pid->ki * candidate_integral
                   - pid->kd * actual_derivative;
  limited_output = My_pid_limit_My(unlimited_output, pid->output_limit);
  integral_output_change = pid->ki * (candidate_integral - previous_integral);

  if (limited_output != unlimited_output &&
      ((unlimited_output > 0.0f && integral_output_change > 0.0f) ||
       (unlimited_output < 0.0f && integral_output_change < 0.0f)))
  {
    /* 新增积分继续推高饱和输出时撤销本周期积分，避免执行器限幅期间积分累积。 */
    candidate_integral = previous_integral;
    unlimited_output = pid->kp * pid->error
                     + pid->ki * candidate_integral
                     - pid->kd * actual_derivative;
    limited_output = My_pid_limit_My(unlimited_output, pid->output_limit);
  }

  pid->integral = candidate_integral;
  pid->output = limited_output;
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
