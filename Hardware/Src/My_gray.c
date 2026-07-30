#include "My_gray.h"
#include "main.h"
#include "My_move.h"
#include "My_timer.h"

volatile My_gray_pid_control_t My_gray_pid_control_My; /* 灰度循迹参数和运行状态，可在调试器中观察。 */

#define MY_GRAY_RAMP_STEPS ((uint16_t)(MY_GRAY_RAMP_TIME_MS / MY_GRAY_CONTROL_PERIOD_MS))
#define MY_GRAY_REVERSE_BRAKE_CYCLES \
  ((uint8_t)(MY_GRAY_REVERSE_BRAKE_TIME_MS / MY_GRAY_CONTROL_PERIOD_MS))

static const int8_t My_gray_position_weight_My[8] =
{
  -7, -5, -3, -1, 1, 3, 5, 7
}; /* 从 Gray_1 到 Gray_8 的位置权重。 */

/**
  * @brief 将浮点值限制在 0 到电机最大 PWM 范围内。
  * @param value 原始限幅值
  * @retval 限幅后的值
  */
static float My_gray_limit_correction_My(float value)
{
  if (value < 0.0f)
  {
    return 0.0f;
  }
  if (value > (float)MY_MOVE_PWM_MAX)
  {
    return (float)MY_MOVE_PWM_MAX;
  }

  return value;
}

/**
  * @brief 计算五次多项式 S 曲线的归一化进度。
  * @param step 当前离散步号
  * @retval 0 到 1 的平滑进度；曲线两端速度和加速度均为零
  */
static float My_gray_s_curve_My(uint16_t step)
{
  float t = (float)step / (float)MY_GRAY_RAMP_STEPS;

  if (t >= 1.0f)
  {
    return 1.0f;
  }

  return t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
}

/**
  * @brief 按 S 型启动加速曲线刷新基础速度。
  * @details 仅在 TIM6 中断调用，不能阻塞；停车由停止接口立即清零，不经过本曲线。
  */
static void My_gray_update_ramp_My(void)
{
  float progress = My_gray_s_curve_My(My_gray_pid_control_My.ramp_step);

  My_gray_pid_control_My.ramp_pwm =
    My_gray_pid_control_My.ramp_start_pwm +
    ((float)My_gray_pid_control_My.base_pwm - My_gray_pid_control_My.ramp_start_pwm) * progress;

  if (My_gray_pid_control_My.ramp_step < MY_GRAY_RAMP_STEPS)
  {
    My_gray_pid_control_My.ramp_step++;
  }
}

/**
  * @brief 读取 8 路灰度传感器的原始数字电平。
  * @retval 8 路电平组成的位图；Gray_1 为最高位，Gray_8 为最低位
  * @note GPIO 高电平对应位值 1，低电平对应位值 0。
  */
uint8_t My_gray_read_My(void)
{
  uint8_t gray_value = 0U; /* 保存从左到右排列的 8 路原始电平。 */

  if (HAL_GPIO_ReadPin(Gray_1_GPIO_Port, Gray_1_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_1_MASK;
  }
  if (HAL_GPIO_ReadPin(Gray_2_GPIO_Port, Gray_2_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_2_MASK;
  }
  if (HAL_GPIO_ReadPin(Gray_3_GPIO_Port, Gray_3_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_3_MASK;
  }
  if (HAL_GPIO_ReadPin(Gray_4_GPIO_Port, Gray_4_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_4_MASK;
  }
  if (HAL_GPIO_ReadPin(Gray_5_GPIO_Port, Gray_5_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_5_MASK;
  }
  if (HAL_GPIO_ReadPin(Gray_6_GPIO_Port, Gray_6_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_6_MASK;
  }
  if (HAL_GPIO_ReadPin(Gray_7_GPIO_Port, Gray_7_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_7_MASK;
  }
  if (HAL_GPIO_ReadPin(Gray_8_GPIO_Port, Gray_8_Pin) == GPIO_PIN_SET)
  {
    gray_value |= MY_GRAY_8_MASK;
  }

  return gray_value;
}

void My_gray_pid_init_My(void)
{
  My_pid_init_My(&My_gray_pid_control_My.pid,
                 20.0f,
                 0.0f,
                 0.0f,
                 100.0f,
                 MY_GRAY_PID_DEFAULT_CORRECTION_LIMIT);
  My_gray_pid_control_My.base_pwm = MY_GRAY_PID_DEFAULT_BASE_PWM;
  My_gray_pid_control_My.correction_limit = MY_GRAY_PID_DEFAULT_CORRECTION_LIMIT;
  My_gray_pid_control_My.error = 0.0f;
  My_gray_pid_control_My.correction = 0.0f;
  My_gray_pid_control_My.raw_value = 0U;
  My_gray_pid_control_My.line_detected = 0U;
  My_gray_pid_control_My.active_level = MY_GRAY_PID_DEFAULT_ACTIVE_LEVEL;
  My_gray_pid_control_My.steering_direction = 1;
  My_gray_pid_control_My.enabled = 0U;
  My_gray_pid_control_My.stop_confirm_count = 0U;
  My_gray_pid_control_My.reverse_brake_cycles = 0U;
  My_gray_pid_control_My.ramp_step = MY_GRAY_RAMP_STEPS;
  My_gray_pid_control_My.ramp_start_pwm = 0.0f;
  My_gray_pid_control_My.ramp_pwm = 0.0f;
}

void My_gray_pid_set_parameters_My(float kp, float ki, float kd)
{
  My_pid_init_My(&My_gray_pid_control_My.pid,
                 kp,
                 ki,
                 kd,
                 100.0f,
                 My_gray_pid_control_My.correction_limit);
}

void My_gray_pid_set_motion_My(int16_t base_pwm, float correction_limit)
{
  if (base_pwm < 0)
  {
    base_pwm = 0;
  }
  if (base_pwm > MY_MOVE_PWM_MAX)
  {
    base_pwm = MY_MOVE_PWM_MAX;
  }

  My_gray_pid_control_My.base_pwm = base_pwm;
  My_gray_pid_control_My.correction_limit = My_gray_limit_correction_My(correction_limit);
  My_gray_pid_control_My.pid.output_limit = My_gray_pid_control_My.correction_limit;
}

void My_gray_pid_set_active_level_My(uint8_t active_level)
{
  My_gray_pid_control_My.active_level = (active_level != 0U) ? 1U : 0U;
}

void My_gray_pid_set_steering_direction_My(int8_t direction)
{
  My_gray_pid_control_My.steering_direction = (direction < 0) ? -1 : 1;
}

void My_gray_pid_start_My(void)
{
  My_pid_reset_My(&My_gray_pid_control_My.pid);
  My_gray_pid_control_My.ramp_start_pwm = My_gray_pid_control_My.ramp_pwm;
  My_gray_pid_control_My.ramp_step = 0U;
  My_gray_pid_control_My.stop_confirm_count = 0U;
  My_gray_pid_control_My.reverse_brake_cycles = 0U;
  My_gray_pid_control_My.enabled = 1U;
}

void My_gray_pid_stop_My(void)
{
  /*
   * 本函数可能由 TIM6 控制中断或主循环临界区调用。先关闭后续循迹更新并把曲线
   * 状态同步归零，再直接清零底盘目标、速度 PID 和四路 PWM；全程不阻塞、不分配
   * 动态内存，返回后不会再由后续周期写入 S 型减速输出。
   */
  My_gray_pid_control_My.enabled = 0U;
  My_gray_pid_control_My.stop_confirm_count = 0U;
  My_gray_pid_control_My.reverse_brake_cycles = 0U;
  My_gray_pid_control_My.ramp_step = MY_GRAY_RAMP_STEPS;
  My_gray_pid_control_My.ramp_start_pwm = 0.0f;
  My_gray_pid_control_My.ramp_pwm = 0.0f;
  My_pid_reset_My(&My_gray_pid_control_My.pid);
  My_move_stop_My();
}

/**
  * @brief 启动任务 2 停止线专用的短时反向制动。
  * @details 本函数仅由 10 ms 控制中断调用。关闭循迹后立即向四轮写入固定反向 PWM，
  *          后续周期只递减制动计数，不再执行灰度 PID；达到设定时间后由周期函数切换
  *          到 TB6612 短路制动。函数不阻塞、不分配动态内存，避免延长中断响应时间。
  * @note 该开环制动没有编码器反馈，强度和时间必须从保守值开始实车标定。
  */
static void My_gray_start_reverse_brake_My(void)
{
  int16_t reverse_pwm = MY_GRAY_REVERSE_BRAKE_PWM; /* 本次实际施加的反向制动强度。 */

  /*
   * 制动力不超过停止线触发前的实际前进 PWM，避免启动初期或静止状态误识别停止线时
   * 产生比原运动指令更强的反向冲击；没有前进输出时直接进入短路制动。
   */
  if (My_gray_pid_control_My.ramp_pwm < (float)reverse_pwm)
  {
    reverse_pwm = (int16_t)My_gray_pid_control_My.ramp_pwm;
  }
  if (reverse_pwm <= 0)
  {
    My_gray_pid_stop_My();
    /* 未施加反向脉冲时，底盘已在本周期完成停车，立即冻结任务 2 用时。 */
    My_timer_stop_My();
    return;
  }

  My_gray_pid_control_My.enabled = 0U;
  My_gray_pid_control_My.stop_confirm_count = 0U;
  My_gray_pid_control_My.reverse_brake_cycles = MY_GRAY_REVERSE_BRAKE_CYCLES;
  My_gray_pid_control_My.ramp_step = MY_GRAY_RAMP_STEPS;
  My_gray_pid_control_My.ramp_start_pwm = 0.0f;
  My_gray_pid_control_My.ramp_pwm = 0.0f;
  My_pid_reset_My(&My_gray_pid_control_My.pid);

  /* 任务 2 停止前只沿车体正方向行驶，因此四轮统一施加反向力矩。 */
  My_move_control_My(-reverse_pwm, -reverse_pwm, -reverse_pwm, -reverse_pwm);
}

void My_gray_pid_update_My(void)
{
  uint8_t active_value; /* 将检测到线统一换算为位值 1 后的灰度位图。 */
  uint8_t track_value;  /* 顺时针循迹保留右侧四路和左侧一路，隔离赛道左侧黑色文字干扰。 */
  uint8_t gray_index;   /* 当前处理的灰度通道索引。 */
  uint8_t active_count = 0U; /* 当前检测到线的通道数量。 */
  int16_t weighted_sum = 0;  /* 所有有效通道的位置权重之和。 */
  float correction;          /* PID 计算后的转向修正量。 */
  float motion_scale;        /* 当前 S 曲线速度占目标基础速度的比例。 */

  /*
   * 停止线触发后，优先完成固定反向制动计时。该状态与循迹互斥，期间不再读取灰度
   * 或改写电机方向；最后一个周期先清零目标、PID 和 PWM，再冻结任务计时，因此最终
   * 用时包含完整反向脉冲。计时状态与主循环共享，由计时模块保证读取快照一致。
   */
  if (My_gray_pid_control_My.reverse_brake_cycles > 0U)
  {
    My_gray_pid_control_My.reverse_brake_cycles--;
    if (My_gray_pid_control_My.reverse_brake_cycles == 0U)
    {
      My_move_stop_My();
      My_timer_stop_My();
    }
    return;
  }

  My_gray_pid_control_My.raw_value = My_gray_read_My();
  active_value = (My_gray_pid_control_My.active_level != 0U)
                   ? My_gray_pid_control_My.raw_value
                   : (uint8_t)(~My_gray_pid_control_My.raw_value);
  track_value = (uint8_t)(active_value & MY_GRAY_TRACK_MASK);

  /*
   * 停车标志独立使用右侧五路：Gray_4~Gray_6、Gray_5~Gray_7 或 Gray_6~Gray_8
   * 任一组三连黑均视为候选。候选必须连续达到 MY_GRAY_STOP_CONFIRM_CYCLES 个
   * 10 ms 周期，
   * 避免左弯宽线或文字边缘在单次采样中形成三连黑而误停；任一周期不满足便重新计数。
   */
  if (My_gray_pid_control_My.enabled != 0U)
  {
    if (((active_value & MY_GRAY_STOP_LEFT_PATTERN) == MY_GRAY_STOP_LEFT_PATTERN) ||
        ((active_value & MY_GRAY_STOP_MIDDLE_PATTERN) == MY_GRAY_STOP_MIDDLE_PATTERN) ||
        ((active_value & MY_GRAY_STOP_RIGHT_PATTERN) == MY_GRAY_STOP_RIGHT_PATTERN))
    {
      if (My_gray_pid_control_My.stop_confirm_count < MY_GRAY_STOP_CONFIRM_CYCLES)
      {
        My_gray_pid_control_My.stop_confirm_count++;
      }
      if (My_gray_pid_control_My.stop_confirm_count >= MY_GRAY_STOP_CONFIRM_CYCLES)
      {
        My_gray_start_reverse_brake_My();
        return;
      }
    }
    else
    {
      My_gray_pid_control_My.stop_confirm_count = 0U;
    }
  }

  /* 只有 Gray_4~Gray_8 参与位置加权，Gray_1~Gray_3 不再影响车辆转向。 */
  for (gray_index = 0U; gray_index < 8U; gray_index++)
  {
    if ((track_value & (uint8_t)(MY_GRAY_1_MASK >> gray_index)) != 0U)
    {
      weighted_sum += My_gray_position_weight_My[gray_index];
      active_count++;
    }
  }

  if (My_gray_pid_control_My.enabled == 0U)
  {
    return;
  }

  My_gray_update_ramp_My();

  if (active_count == 0U)
  {
    /*
     * 当前采样未检测到黑线，只更新丢线状态并保持上一周期的误差、PID 状态和
     * 四轮 PWM 输出。这样车辆会沿最后一次有效修正方向继续运动，直到传感器
     * 再次检测到黑线后恢复位置计算。首次启动前电机已由运动模块保持停止，
     * 因此尚无有效循迹输出时进入本分支不会产生新的运动指令。
     *
     * 此策略不设置丢线超时：传感器持续故障时车辆会持续执行最后指令，实车
     * 使用时必须依靠人工急停或上层任务状态机终止运动。
     */
    My_gray_pid_control_My.line_detected = 0U;
    return;
  }

  My_gray_pid_control_My.line_detected = 1U;
  My_gray_pid_control_My.error = (float)weighted_sum / (float)active_count;
  correction = My_pid_calc_position_My(&My_gray_pid_control_My.pid,
                                       0.0f,
                                       My_gray_pid_control_My.error);
  correction *= (float)My_gray_pid_control_My.steering_direction;
  /* 转向量与前进速度使用同一曲线比例，防止启动初期仅转向输出突然增大。 */
  motion_scale = My_gray_pid_control_My.base_pwm > 0
                   ? My_gray_pid_control_My.ramp_pwm /
                     (float)My_gray_pid_control_My.base_pwm
                   : 0.0f;
  correction *= motion_scale;
  My_gray_pid_control_My.correction = correction;

  My_move_mecanum_inverse_My(My_gray_pid_control_My.ramp_pwm,
                             0.0f,
                             correction);
  My_move_velocity_pid_update_My();
}
