#include "My_steering.h"

#include "My_encoder.h"
#include "main.h"
#include "tim.h"

volatile My_steering_control_t My_steering_control_My; /* TIM6 中断更新、主循环和调试器观察的转向控制状态。 */

/**
  * @brief 将位置目标约束在当前软件限位范围内。
  * @param target_count 原始目标累计计数
  * @retval 限幅后的目标累计计数
  */
static int32_t My_steering_limit_target_My(int32_t target_count)
{
  if (target_count > My_steering_control_My.max_target_count)
  {
    return My_steering_control_My.max_target_count;
  }
  if (target_count < My_steering_control_My.min_target_count)
  {
    return My_steering_control_My.min_target_count;
  }

  return target_count;
}

/**
  * @brief 根据当前零点和角度限位刷新允许的目标计数范围。
  * @note 调用方必须已屏蔽中断，避免 TIM6 位置环观察到只更新一半的限位状态。
  */
static void My_steering_refresh_limit_My(void)
{
  int32_t limit_count = My_encoder_output_angle_to_count_My(
    My_steering_control_My.angle_limit_deg); /* 正角度限位对应的计数绝对值。 */

  if (limit_count < 0)
  {
    limit_count = -limit_count;
  }

  My_steering_control_My.min_target_count = My_steering_control_My.zero_count - limit_count;
  My_steering_control_My.max_target_count = My_steering_control_My.zero_count + limit_count;
  My_steering_control_My.target_count = My_steering_limit_target_My(
    My_steering_control_My.target_count);
}

/**
  * @brief 将浮点 PID 输出转换为 TIM4_CH3 的有符号 PWM 指令。
  * @param value PID 浮点输出
  * @retval 限制在 -999～999 范围内的有符号 PWM
  */
static int16_t My_steering_float_to_pwm_My(float value)
{
  if (value > (float)MY_STEERING_PWM_MAX)
  {
    return MY_STEERING_PWM_MAX;
  }
  if (value < -(float)MY_STEERING_PWM_MAX)
  {
    return -MY_STEERING_PWM_MAX;
  }

  return (value >= 0.0f) ? (int16_t)(value + 0.5f) : (int16_t)(value - 0.5f);
}

/**
  * @brief 对浮点控制量做正负对称限幅。
  * @param value 待限制的控制量
  * @param limit 正的限幅绝对值；小于等于零时不限制
  * @retval 限幅后的控制量
  */
static float My_steering_limit_float_My(float value, float limit)
{
  if (limit <= 0.0f)
  {
    return value;
  }
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

/**
  * @brief 计算钢球平衡模式下带抗积分累积的位置控制量。
  * @param error_count 当前目标计数减实际计数
  * @param actual_delta_count 最近 10 ms 实际位置变化量
  * @retval 已按 PID 输出范围限幅的浮点 PWM
  * @details 微分项只使用实际位置变化量，避免外环每周期更新目标时产生微分冲击；
  *          输出饱和且误差仍推动同方向饱和时撤销本周期积分，防止机构受静摩擦
  *          阻塞期间积累过量控制量。误差换向时清零积分，降低越过目标后的反冲。
  */
static float My_steering_calc_balance_pid_My(int32_t error_count,
                                             int32_t actual_delta_count)
{
  volatile My_pid_t *pid = &My_steering_control_My.pid; /* 平衡模式沿用在线可观察的 PID 参数和状态。 */
  float error = (float)error_count; /* 当前编码器计数误差的浮点表示。 */
  float previous_integral = pid->integral; /* 本周期积分前的快照，饱和时用于回退。 */
  float candidate_integral; /* 加入当前误差并完成限幅后的候选积分。 */
  float output;             /* 尚未完成抗饱和判定的控制器输出。 */
  float limited_output;     /* 按 PID 输出范围限制后的控制器输出。 */

  if ((error > 0.0f && pid->last_error < 0.0f) ||
      (error < 0.0f && pid->last_error > 0.0f))
  {
    /* 误差已经越过零点，旧方向积分会加剧反冲，因此从本周期开始重新建立静态偏置。 */
    previous_integral = 0.0f;
  }

  candidate_integral = My_steering_limit_float_My(
    previous_integral + error,
    pid->integral_limit);
  output = pid->kp * error
         + pid->ki * candidate_integral
         - pid->kd * (float)actual_delta_count;
  limited_output = My_steering_limit_float_My(output, pid->output_limit);

  if (limited_output != output &&
      ((output > 0.0f && error > 0.0f) ||
       (output < 0.0f && error < 0.0f)))
  {
    /* 饱和方向与误差方向一致时，本周期积分只会让饱和更严重，回退到积分前状态。 */
    candidate_integral = previous_integral;
    output = pid->kp * error
           + pid->ki * candidate_integral
           - pid->kd * (float)actual_delta_count;
    limited_output = My_steering_limit_float_My(output, pid->output_limit);
  }

  pid->error = error;
  pid->integral = candidate_integral;
  pid->output = limited_output;
  pid->prev_error = pid->last_error;
  pid->last_error = error;
  return limited_output;
}

/**
  * @brief 把平衡模式控制量抬升到当前方向可克服摩擦的最小 PWM。
  * @param pid_output 已限幅的浮点 PID 输出
  * @param error_count 当前目标计数减实际计数
  * @param actual_delta_count 最近 10 ms 实际位置变化量
  * @retval 加入静止启动力或运动摩擦补偿后的有符号 PWM
  * @details 正反方向分别保留独立标定值。编码器几乎不动时使用启动力，已经运动
  *          时使用较低的运行补偿；只抬升幅值，不改变 PID 已决定的制动方向。
  */
static int16_t My_steering_apply_balance_friction_My(float pid_output,
                                                     int32_t error_count,
                                                     int32_t actual_delta_count)
{
  int16_t command = My_steering_float_to_pwm_My(pid_output); /* PID 原始有符号 PWM。 */
  int32_t delta_abs = (actual_delta_count < 0) ? -actual_delta_count : actual_delta_count; /* 判断机构是否已经运动。 */
  int16_t minimum_pwm; /* 当前方向和运动状态对应的最小有效 PWM。 */
  int16_t command_abs; /* 原始 PWM 的绝对值。 */

  if (command == 0)
  {
    if (error_count > 0)
    {
      command = 1;
    }
    else if (error_count < 0)
    {
      command = -1;
    }
    else
    {
      My_steering_control_My.friction_compensation_active = 0U;
      return 0;
    }
  }

  if (command > 0)
  {
    minimum_pwm = (delta_abs <= MY_STEERING_BALANCE_MOVING_DELTA_COUNT)
      ? MY_STEERING_BALANCE_BREAKAWAY_PWM_POS
      : MY_STEERING_BALANCE_RUNNING_PWM_POS;
    command_abs = command;
  }
  else
  {
    minimum_pwm = (delta_abs <= MY_STEERING_BALANCE_MOVING_DELTA_COUNT)
      ? MY_STEERING_BALANCE_BREAKAWAY_PWM_NEG
      : MY_STEERING_BALANCE_RUNNING_PWM_NEG;
    command_abs = (int16_t)-command;
  }

  if (command_abs < minimum_pwm)
  {
    My_steering_control_My.friction_compensation_active = 1U;
    return (command > 0) ? minimum_pwm : (int16_t)-minimum_pwm;
  }

  My_steering_control_My.friction_compensation_active = 0U;
  return command;
}

/**
  * @brief 向转向电机驱动器写入方向和 PWM。
  * @param command 有符号 PWM，正负表示逻辑方向
  * @details 每次先把 TIM4_CH3 比较值清零，再切换 Steering_1A/1B，防止带载
  *          直接换向。默认正方向为 1A=1、1B=0；motor_direction 可整体反转。
  */
static void My_steering_write_output_My(int16_t command)
{
  int32_t mapped_command = (int32_t)command * (int32_t)My_steering_control_My.motor_direction; /* 按当前电机接线方向映射后的有符号 PWM 指令。 */
  uint16_t duty; /* 写入 TIM4_CH3 比较寄存器的 PWM 占空比绝对值。 */

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);

  if (mapped_command > 0)
  {
    HAL_GPIO_WritePin(Steering_1A_GPIO_Port, Steering_1A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Steering_1B_GPIO_Port, Steering_1B_Pin, GPIO_PIN_RESET);
    duty = (uint16_t)mapped_command;
  }
  else if (mapped_command < 0)
  {
    HAL_GPIO_WritePin(Steering_1A_GPIO_Port, Steering_1A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Steering_1B_GPIO_Port, Steering_1B_Pin, GPIO_PIN_SET);
    duty = (uint16_t)(-mapped_command);
  }
  else
  {
    /* 两个方向输入均为低电平时保持无驱动输出，避免停机状态持续耗能。 */
    HAL_GPIO_WritePin(Steering_1A_GPIO_Port, Steering_1A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Steering_1B_GPIO_Port, Steering_1B_Pin, GPIO_PIN_RESET);
    duty = 0U;
  }

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty);
}

HAL_StatusTypeDef My_steering_init_My(void)
{
  My_steering_control_My.enabled = 0U;
  My_steering_control_My.motor_direction = 1;
  My_steering_control_My.actual_count = My_encoder_get_total_count_My();
  My_steering_control_My.previous_actual_count = My_steering_control_My.actual_count;
  My_steering_control_My.zero_count = My_steering_control_My.actual_count;
  My_steering_control_My.target_count = My_steering_control_My.actual_count;
  My_steering_control_My.target_angle_deg = 0.0f;
  My_steering_control_My.actual_angle_deg = 0.0f;
  My_steering_control_My.angle_limit_deg = 0.0f;
  My_steering_refresh_limit_My();
  My_steering_control_My.pwm_output = 0;
  My_steering_control_My.actual_delta_count = 0;
  My_steering_control_My.zero_ready = 0U;
  My_steering_control_My.in_deadband = 0U;
  My_steering_control_My.balance_mode = 0U;
  My_steering_control_My.friction_compensation_active = 0U;

  /* 默认参数仅建立保守的计数位置 P 环；实车使用前应根据负载和编码器分辨率整定。 */
  My_pid_init_My(&My_steering_control_My.pid, 0.10f, 0.01f, 0.1f, 5000.0f, 300.0f);
  My_steering_write_output_My(0);

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void My_steering_set_pid_My(float kp,
                            float ki,
                            float kd,
                            float integral_limit,
                            float output_limit)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 进入 PID 参数临界区前保存的全局中断屏蔽状态。 */

  if (output_limit <= 0.0f || output_limit > (float)MY_STEERING_PWM_MAX)
  {
    output_limit = (float)MY_STEERING_PWM_MAX;
  }

  /* PID 状态由 TIM6 中断访问，参数与历史状态必须在同一临界区内整体更新。 */
  __disable_irq();
  My_pid_init_My(&My_steering_control_My.pid,
                 kp,
                 ki,
                 kd,
                 integral_limit,
                 output_limit);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

void My_steering_set_direction_My(int8_t direction)
{
  My_steering_control_My.motor_direction = (direction < 0) ? -1 : 1;
}

void My_steering_set_balance_mode_My(uint8_t enabled)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 保存调用前的全局中断屏蔽状态。 */

  /*
   * 模式标志、速度差分起点和 PID 历史会被 TIM6 同时使用，必须在同一短临界区
   * 提交。这里只进行自然对齐赋值和状态清零，不访问阻塞外设。
   */
  __disable_irq();
  My_steering_control_My.balance_mode = (enabled != 0U) ? 1U : 0U;
  My_steering_control_My.previous_actual_count = My_steering_control_My.actual_count;
  My_steering_control_My.actual_delta_count = 0;
  My_steering_control_My.in_deadband = 0U;
  My_steering_control_My.friction_compensation_active = 0U;
  My_pid_reset_My(&My_steering_control_My.pid);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

void My_steering_set_zero_My(void)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 建立转向零点前保存的全局中断屏蔽状态。 */

  /*
   * 零点建立会同时改变目标和限位基准，因此必须与 TIM6 位置环原子切换。
   * 先禁止驱动并清零 PWM，确保人工调平期间不会突然恢复旧的位置目标。
   */
  __disable_irq();
  My_steering_control_My.enabled = 0U;
  My_steering_control_My.pwm_output = 0;
  My_steering_write_output_My(0);
  My_steering_control_My.actual_count = My_encoder_get_total_count_My();
  My_steering_control_My.previous_actual_count = My_steering_control_My.actual_count;
  My_steering_control_My.actual_delta_count = 0;
  My_steering_control_My.zero_count = My_steering_control_My.actual_count;
  My_steering_control_My.target_count = My_steering_control_My.zero_count;
  My_steering_control_My.target_angle_deg = 0.0f;
  My_steering_control_My.actual_angle_deg = 0.0f;
  My_steering_control_My.angle_limit_deg = 0.0f;
  My_steering_refresh_limit_My();
  My_steering_control_My.zero_ready = 1U;
  My_steering_control_My.in_deadband = 0U;
  My_steering_control_My.friction_compensation_active = 0U;
  My_pid_reset_My(&My_steering_control_My.pid);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

HAL_StatusTypeDef My_steering_set_angle_limit_My(float limit_deg)
{
  uint32_t interrupt_mask; /* 更新角度软限位前保存的全局中断屏蔽状态。 */

  if (limit_deg <= 0.0f)
  {
    return HAL_ERROR;
  }

  interrupt_mask = __get_PRIMASK();
  __disable_irq();
  My_steering_control_My.angle_limit_deg = limit_deg;
  My_steering_refresh_limit_My();
  My_steering_control_My.target_angle_deg = My_encoder_count_to_output_angle_My(
    My_steering_control_My.target_count - My_steering_control_My.zero_count);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }

  return HAL_OK;
}

HAL_StatusTypeDef My_steering_start_My(int32_t target_count)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 启动位置环前保存的全局中断屏蔽状态。 */

  /* 增量编码器未建立绝对零点或未配置活动范围时，禁止产生任何电机输出。 */
  if (My_steering_control_My.zero_ready == 0U ||
      My_steering_control_My.angle_limit_deg <= 0.0f)
  {
    return HAL_ERROR;
  }

  /* 启动状态与 PID 历史由 TIM6 中断共同使用，需原子完成整个状态切换。 */
  __disable_irq();
  My_pid_reset_My(&My_steering_control_My.pid);
  My_steering_control_My.target_count = My_steering_limit_target_My(target_count);
  My_steering_control_My.target_angle_deg = My_encoder_count_to_output_angle_My(
    My_steering_control_My.target_count - My_steering_control_My.zero_count);
  My_steering_control_My.in_deadband = 0U;
  My_steering_control_My.previous_actual_count = My_steering_control_My.actual_count;
  My_steering_control_My.actual_delta_count = 0;
  My_steering_control_My.friction_compensation_active = 0U;
  My_steering_control_My.enabled = 1U;
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }

  return HAL_OK;
}

void My_steering_set_target_My(int32_t target_count)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 更新目标计数前保存的全局中断屏蔽状态。 */

  /* 目标计数和调试角度必须作为同一个状态更新，避免观察到二者来自不同指令。 */
  __disable_irq();
  My_steering_control_My.target_count = My_steering_limit_target_My(target_count);
  My_steering_control_My.target_angle_deg = My_encoder_count_to_output_angle_My(
    My_steering_control_My.target_count - My_steering_control_My.zero_count);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

HAL_StatusTypeDef My_steering_start_angle_My(float target_angle_deg)
{
  int32_t relative_count = My_encoder_output_angle_to_count_My(target_angle_deg); /* 目标输出轴角度相对零点换算得到的编码器计数。 */

  return My_steering_start_My(My_steering_control_My.zero_count + relative_count);
}

HAL_StatusTypeDef My_steering_start_from_fixed_position_My(float target_angle_deg,
                                                          float limit_deg)
{
  float target_abs = (target_angle_deg < 0.0f) ? -target_angle_deg : target_angle_deg; /* 固定起点目标角度的绝对值，用于校验软限位。 */

  /* 安全限位必须覆盖完整目标行程，否则启动后目标会被静默截断而无法到达水平位置。 */
  if (limit_deg <= 0.0f || target_abs > limit_deg)
  {
    return HAL_ERROR;
  }

  /*
   * 上电固定位置作为本次相对零点。建立零点会保持 PWM 为零，随后配置实际允许
   * 行程，最后才使能位置 PID，整个顺序避免未建立参考时产生电机输出。
   */
  My_steering_set_zero_My();
  if (My_steering_set_angle_limit_My(limit_deg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return My_steering_start_angle_My(target_angle_deg);
}

void My_steering_set_target_angle_My(float target_angle_deg)
{
  int32_t relative_count = My_encoder_output_angle_to_count_My(target_angle_deg); /* 新目标角度相对零点换算得到的编码器计数。 */

  My_steering_set_target_My(My_steering_control_My.zero_count + relative_count);
}

float My_steering_get_actual_angle_My(void)
{
  return My_steering_control_My.actual_angle_deg;
}

void My_steering_stop_My(void)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 停止位置环并同步状态前保存的全局中断屏蔽状态。 */

  /* 先禁止周期输出，再锁定当前位置并清除 PID，确保退出临界区后不会恢复旧指令。 */
  __disable_irq();
  My_steering_control_My.enabled = 0U;
  My_steering_control_My.actual_count = My_encoder_get_total_count_My();
  My_steering_control_My.target_count = My_steering_control_My.actual_count;
  My_steering_control_My.target_angle_deg = My_encoder_count_to_output_angle_My(
    My_steering_control_My.target_count - My_steering_control_My.zero_count);
  My_steering_control_My.actual_angle_deg = My_steering_control_My.target_angle_deg;
  My_steering_control_My.pwm_output = 0;
  My_steering_control_My.in_deadband = 0U;
  My_steering_control_My.previous_actual_count = My_steering_control_My.actual_count;
  My_steering_control_My.actual_delta_count = 0;
  My_steering_control_My.friction_compensation_active = 0U;
  My_pid_reset_My(&My_steering_control_My.pid);
  My_steering_write_output_My(0);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

void My_steering_update_10ms_My(void)
{
  float pid_output; /* 本周期位置式 PID 计算得到的浮点 PWM 指令。 */
  int32_t error_count;  /* 当前目标与反馈的有符号位置误差，单位为编码器计数。 */
  int32_t error_abs;    /* 用于死区判断的误差绝对值。 */
  int32_t stop_count;   /* 0.5 度停止死区换算后的编码器计数。 */
  int32_t resume_count; /* 0.7 度恢复阈值换算后的编码器计数。 */
  int32_t actual_delta_count; /* 平衡模式使用的本周期实际位置增量，单位为编码器计数。 */

  /* 编码器已在本次 TIM6 中断前半段采样，此处读取同一周期的软件累计位置。 */
  My_steering_control_My.actual_count = My_encoder_get_total_count_My();
  actual_delta_count = My_steering_control_My.actual_count -
    My_steering_control_My.previous_actual_count;
  My_steering_control_My.previous_actual_count = My_steering_control_My.actual_count;
  My_steering_control_My.actual_delta_count = actual_delta_count;
  My_steering_control_My.actual_angle_deg = My_encoder_count_to_output_angle_My(
    My_steering_control_My.actual_count - My_steering_control_My.zero_count);

  if (My_steering_control_My.enabled == 0U)
  {
    My_steering_control_My.pwm_output = 0;
    My_steering_write_output_My(0);
    return;
  }

  /*
   * 本段运行在 TIM6 中断内，只进行定长整数比较、PID 状态清零和外设寄存器写入，
   * 不调用阻塞函数或分配动态内存。target_count 可能由主循环在临界区更新，
   * actual_count 则来自本次中断前半段的编码器采样，因此这里读取到的是一致状态。
   */
  error_count = My_steering_control_My.target_count - My_steering_control_My.actual_count;
  error_abs = (error_count < 0) ? -error_count : error_count;

  if (My_steering_control_My.balance_mode != 0U)
  {
    stop_count = My_encoder_output_angle_to_count_My(MY_STEERING_BALANCE_DEADBAND_DEG);

    /*
     * 钢球平衡需要平台能响应远小于普通定位死区的角度修正。微死区内清除积分并
     * 停止输出，避免编码器噪声驱动电机；微死区外持续闭环，不使用滞回断电，
     * 并通过最小 PWM 补偿解决小误差下克服不了静摩擦的问题。
     */
    if (error_abs <= stop_count)
    {
      My_pid_reset_My(&My_steering_control_My.pid);
      My_steering_control_My.pwm_output = 0;
      My_steering_control_My.in_deadband = 1U;
      My_steering_control_My.friction_compensation_active = 0U;
      My_steering_write_output_My(0);
      return;
    }

    My_steering_control_My.in_deadband = 0U;
    pid_output = My_steering_calc_balance_pid_My(error_count, actual_delta_count);
    My_steering_control_My.pwm_output = My_steering_apply_balance_friction_My(
      pid_output,
      error_count,
      actual_delta_count);
    My_steering_write_output_My(My_steering_control_My.pwm_output);
    return;
  }

  stop_count = My_encoder_output_angle_to_count_My(MY_STEERING_STOP_DEADBAND_DEG);
  resume_count = My_encoder_output_angle_to_count_My(MY_STEERING_RESUME_DEADBAND_DEG);

  /*
   * 已到位时保持 PID 历史和 PWM 为零，彻底阻断编码器微小静差的长期积分累计。
   * 只有误差严格超过 0.7 度才退出保持状态；阈值内不驱动意味着负载可能产生
   * 允许范围内的静态偏差，这是消除长时间积分突跳所接受的控制取舍。
   */
  if (My_steering_control_My.in_deadband != 0U)
  {
    if (error_abs <= resume_count)
    {
      My_pid_reset_My(&My_steering_control_My.pid);
      My_steering_control_My.pwm_output = 0;
      My_steering_write_output_My(0);
      return;
    }

    My_steering_control_My.in_deadband = 0U;
    My_pid_reset_My(&My_steering_control_My.pid);
  }

  /*
   * 首次进入 0.5 度停止死区时立即清除已累计的积分和微分历史，再关闭电机输出。
   * in_deadband 由本中断独占写入，主循环仅用于调试观察，不存在读改写竞态。
   */
  if (error_abs <= stop_count)
  {
    My_steering_control_My.in_deadband = 1U;
    My_pid_reset_My(&My_steering_control_My.pid);
    My_steering_control_My.pwm_output = 0;
    My_steering_write_output_My(0);
    return;
  }

  /* 固定 10 ms 执行一次位置式 PID，并把有限幅的结果转换成有符号 PWM。 */
  pid_output = My_pid_calc_position_My(&My_steering_control_My.pid,
                                      (float)My_steering_control_My.target_count,
                                      (float)My_steering_control_My.actual_count);
  My_steering_control_My.pwm_output = My_steering_float_to_pwm_My(pid_output);
  My_steering_write_output_My(My_steering_control_My.pwm_output);
}
