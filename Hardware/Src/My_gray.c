#include "My_gray.h"
#include "main.h"
#include "My_move.h"
#include "My_timer.h"

volatile My_gray_pid_control_t My_gray_pid_control_My; /* 灰度循迹参数和运行状态，可在调试器中观察。 */

/* S 型速度曲线包含的 10 毫秒离散控制步数。 */
#define MY_GRAY_RAMP_STEPS ((uint16_t)(MY_GRAY_RAMP_TIME_MS / MY_GRAY_CONTROL_PERIOD_MS))
/* 停车标志触发后需要执行的反向制动控制周期数。 */
#define MY_GRAY_REVERSE_BRAKE_CYCLES \
  ((uint8_t)(MY_GRAY_REVERSE_BRAKE_TIME_MS / MY_GRAY_CONTROL_PERIOD_MS))
/* 任务 5 终点 S 型减速包含的 10 毫秒离散控制步数。 */
#define MY_GRAY_STABLE_DECEL_STEPS \
  ((uint16_t)(MY_GRAY_STABLE_DECEL_TIME_MS / MY_GRAY_CONTROL_PERIOD_MS))

static const int8_t My_gray_position_weight_My[8] = /* Gray_1～Gray_8 对应的循迹横向位置权重。 */
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
  * @param total_steps 当前曲线的总步数
  * @retval 0 到 1 的平滑进度；曲线两端速度和加速度均为零
  */
static float My_gray_s_curve_My(uint16_t step, uint16_t total_steps)
{
  float t = (float)step / (float)total_steps; /* 当前曲线步号归一化后的时间进度，范围为 0～1。 */

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
  float progress = My_gray_s_curve_My(My_gray_pid_control_My.ramp_step,
                                      MY_GRAY_RAMP_STEPS); /* 当前 S 曲线计算出的平滑速度比例。 */

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
  My_gray_pid_control_My.reverse_brake_speed = 0.0f;
  My_gray_pid_control_My.ramp_step = MY_GRAY_RAMP_STEPS;
  My_gray_pid_control_My.ramp_start_pwm = 0.0f;
  My_gray_pid_control_My.ramp_pwm = 0.0f;
  My_gray_pid_control_My.stable_stop_enabled = 0U;
  My_gray_pid_control_My.stable_decelerating = 0U;
  My_gray_pid_control_My.stable_decel_step = MY_GRAY_STABLE_DECEL_STEPS;
  My_gray_pid_control_My.stable_decel_start_pwm = 0.0f;
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
  My_gray_pid_control_My.reverse_brake_speed = 0.0f;
  My_gray_pid_control_My.stable_stop_enabled = 0U;
  My_gray_pid_control_My.stable_decelerating = 0U;
  My_gray_pid_control_My.stable_decel_step = MY_GRAY_STABLE_DECEL_STEPS;
  My_gray_pid_control_My.stable_decel_start_pwm = 0.0f;
  My_gray_pid_control_My.enabled = 1U;
}

void My_gray_pid_start_stable_My(void)
{
  /*
   * 先复用任务 2 的 PID、终点确认和启动 S 曲线复位，再切换为任务 5 专用的低速
   * 与平滑停车模式。调用方已屏蔽 TIM6，中间状态不会被控制中断观察到。
   */
  My_gray_pid_start_My();
  My_gray_pid_control_My.base_pwm = MY_GRAY_STABLE_BASE_PWM;
  My_gray_pid_control_My.stable_stop_enabled = 1U;
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
  My_gray_pid_control_My.reverse_brake_speed = 0.0f;
  My_gray_pid_control_My.ramp_step = MY_GRAY_RAMP_STEPS;
  My_gray_pid_control_My.ramp_start_pwm = 0.0f;
  My_gray_pid_control_My.ramp_pwm = 0.0f;
  My_gray_pid_control_My.stable_stop_enabled = 0U;
  My_gray_pid_control_My.stable_decelerating = 0U;
  My_gray_pid_control_My.stable_decel_step = MY_GRAY_STABLE_DECEL_STEPS;
  My_gray_pid_control_My.stable_decel_start_pwm = 0.0f;
  My_pid_reset_My(&My_gray_pid_control_My.pid);
  My_move_stop_My();
}

void My_gray_pid_start_external_My(void)
{
  /*
   * 任务 4 的速度由应用层 5 s S 型曲线逐周期给出，本接口只复位灰度位置环和
   * 任务 2 的停车/制动残留状态。enabled 保持 0，防止 My_gray_pid_update_My()
   * 在同一个 TIM6 周期误进入任务 2 的停止线、13 秒降速或内部启动曲线流程。
   */
  My_pid_reset_My(&My_gray_pid_control_My.pid);
  My_gray_pid_control_My.error = 0.0f;
  My_gray_pid_control_My.correction = 0.0f;
  My_gray_pid_control_My.line_detected = 0U;
  My_gray_pid_control_My.enabled = 0U;
  My_gray_pid_control_My.stop_confirm_count = 0U;
  My_gray_pid_control_My.reverse_brake_cycles = 0U;
  My_gray_pid_control_My.reverse_brake_speed = 0.0f;
  My_gray_pid_control_My.ramp_step = MY_GRAY_RAMP_STEPS;
  My_gray_pid_control_My.ramp_start_pwm = 0.0f;
  My_gray_pid_control_My.ramp_pwm = 0.0f;
  My_gray_pid_control_My.stable_stop_enabled = 0U;
  My_gray_pid_control_My.stable_decelerating = 0U;
  My_gray_pid_control_My.stable_decel_step = MY_GRAY_STABLE_DECEL_STEPS;
  My_gray_pid_control_My.stable_decel_start_pwm = 0.0f;
}

/**
  * @brief 启动任务 2 停止线专用的短时反向制动。
  * @details 本函数仅由 10 ms 控制中断调用。关闭循迹后向四轮下发固定反向速度目标，
  *          当前周期和后续每个周期都通过编码器速度 PID 计算 PWM；达到设定时间后
  *          切换到 TB6612 短路制动。函数不阻塞、不分配动态内存。
  * @note 反向速度目标和持续时间必须从保守值开始实车标定。
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
  My_gray_pid_control_My.reverse_brake_speed = (float)reverse_pwm;
  My_gray_pid_control_My.ramp_step = MY_GRAY_RAMP_STEPS;
  My_gray_pid_control_My.ramp_start_pwm = 0.0f;
  My_gray_pid_control_My.ramp_pwm = 0.0f;
  My_gray_pid_control_My.stable_decelerating = 0U;
  My_gray_pid_control_My.stable_decel_step = MY_GRAY_STABLE_DECEL_STEPS;
  My_gray_pid_control_My.stable_decel_start_pwm = 0.0f;
  My_pid_reset_My(&My_gray_pid_control_My.pid);

  /* 任务 2 停止前只沿车体正方向行驶，四轮统一通过速度 PID 跟踪反向速度目标。 */
  My_move_mecanum_inverse_My(-My_gray_pid_control_My.reverse_brake_speed, 0.0f, 0.0f);
  My_move_velocity_pid_update_My();
}

void My_gray_pid_update_external_pwm_My(float base_pwm)
{
  uint8_t active_value; /* 将当前采样按配置电平换算为“检测到线为 1”的位图。 */
  uint8_t track_value;  /* 只保留任务 2 已验证的 Gray_4～Gray_8 循迹通道。 */
  uint8_t gray_index;   /* 当前参与加权统计的灰度通道索引。 */
  uint8_t active_count = 0U; /* 当前检测到赛道线的有效通道数量。 */
  int16_t weighted_sum = 0;  /* 有效通道权重累加值，用于得到灰度横向误差。 */
  float limited_pwm;         /* 限幅后的任务 4 当前基础 PWM。 */
  float correction;          /* 位置 PID 输出并按方向和速度比例修正后的转向量。 */
  float motion_scale;        /* 当前外部基础速度相对配置基础速度的比例。 */

  if (base_pwm < 0.0f)
  {
    limited_pwm = 0.0f;
  }
  else if (base_pwm > (float)MY_MOVE_PWM_MAX)
  {
    limited_pwm = (float)MY_MOVE_PWM_MAX;
  }
  else
  {
    limited_pwm = base_pwm;
  }

  My_gray_pid_control_My.ramp_pwm = limited_pwm;
  My_gray_pid_control_My.raw_value = My_gray_read_My();
  active_value = (My_gray_pid_control_My.active_level != 0U)
                   ? My_gray_pid_control_My.raw_value
                   : (uint8_t)(~My_gray_pid_control_My.raw_value);
  track_value = (uint8_t)(active_value & MY_GRAY_TRACK_MASK);

  /* 任务 4 沿用 Gray_4～Gray_8 的循迹权重，避免左侧文字或无关黑区干扰转向。 */
  for (gray_index = 0U; gray_index < 8U; gray_index++)
  {
    if ((track_value & (uint8_t)(MY_GRAY_1_MASK >> gray_index)) != 0U)
    {
      weighted_sum += My_gray_position_weight_My[gray_index];
      active_count++;
    }
  }

  if (limited_pwm <= 0.0f)
  {
    /*
     * S 曲线起点和终点速度为零时必须主动清零四轮目标，避免沿用上一周期丢线保持
     * 的输出；PID 状态保留给后续非零速度周期继续平滑修正。
     */
    My_gray_pid_control_My.line_detected = (active_count > 0U) ? 1U : 0U;
    My_gray_pid_control_My.correction = 0.0f;
    My_move_mecanum_inverse_My(0.0f, 0.0f, 0.0f);
    My_move_velocity_pid_update_My();
    return;
  }

  if (active_count == 0U)
  {
    /*
     * 任务 4 的上层状态机严格限定 5 s 运行时间。丢线时不生成新的转向量，保持
     * 上一次有效速度目标等待重新捕获灰度线，但仍使用本周期编码器反馈更新四轮
     * 速度 PID；若传感器持续无效，5 s 边界仍会强制停车。
     */
    My_gray_pid_control_My.line_detected = 0U;
    My_move_velocity_pid_update_My();
    return;
  }

  My_gray_pid_control_My.line_detected = 1U;
  My_gray_pid_control_My.error = (float)weighted_sum / (float)active_count;
  correction = My_pid_calc_position_My(&My_gray_pid_control_My.pid,
                                       0.0f,
                                       My_gray_pid_control_My.error);
  correction *= (float)My_gray_pid_control_My.steering_direction;

  /*
   * 外部速度越低，转向修正同步减小，避免 S 曲线起步和临停阶段出现只有转向、
   * 几乎没有前进速度的突变。比例上限为 1，保持原 PID 输出限幅语义不被放大。
   */
  motion_scale = My_gray_pid_control_My.base_pwm > 0
                   ? limited_pwm / (float)My_gray_pid_control_My.base_pwm
                   : 0.0f;
  if (motion_scale > 1.0f)
  {
    motion_scale = 1.0f;
  }
  correction *= motion_scale;
  My_gray_pid_control_My.correction = correction;

  My_move_mecanum_inverse_My(limited_pwm, 0.0f, correction);
  My_move_velocity_pid_update_My();
}

void My_gray_pid_update_My(void)
{
  uint8_t active_value; /* 将检测到线统一换算为位值 1 后的灰度位图。 */
  uint8_t track_value;  /* 顺时针循迹保留右侧四路和左侧一路，隔离赛道左侧黑色文字干扰。 */
  uint8_t gray_index;   /* 当前处理的灰度通道索引。 */
  uint8_t active_count = 0U; /* 当前检测到线的通道数量。 */
  int16_t weighted_sum = 0;  /* 所有有效通道的位置权重之和。 */
  uint32_t elapsed_ms;       /* 当前中断读取的任务 2 运行时间快照，单位毫秒，用于判断是否进入低速段。 */
  float correction;          /* PID 计算后的转向修正量。 */
  float motion_scale;        /* 当前 S 曲线速度占目标基础速度的比例。 */
  float decel_progress;      /* 任务 5 终点减速曲线的归一化进度。 */

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
    else
    {
      /*
       * 反向制动期间每个 TIM6 周期都使用最新编码器速度重新计算四轮 PWM，
       * 不保持旧 PWM，也不允许固定占空比绕过速度环。
       */
      My_move_mecanum_inverse_My(-My_gray_pid_control_My.reverse_brake_speed, 0.0f, 0.0f);
      My_move_velocity_pid_update_My();
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
  if (My_gray_pid_control_My.enabled != 0U &&
      My_gray_pid_control_My.stable_decelerating == 0U)
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
        if (My_gray_pid_control_My.stable_stop_enabled != 0U)
        {
          /*
           * 任务 5 从终点确认瞬间的实际曲线 PWM 开始五次多项式减速，首周期输出
           * 与前一周期连续，避免速度阶跃扰动钢球；减速期间锁定终点状态，不重复计数。
           */
          My_gray_pid_control_My.stable_decelerating = 1U;
          My_gray_pid_control_My.stable_decel_step = 0U;
          My_gray_pid_control_My.stable_decel_start_pwm =
            My_gray_pid_control_My.ramp_pwm;
          My_gray_pid_control_My.stop_confirm_count = 0U;
        }
        else
        {
          /* 任务 2 保持原 50 ms 反向制动和计时停止顺序，不改变既有验收行为。 */
          My_gray_start_reverse_brake_My();
          return;
        }
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

  if (My_gray_pid_control_My.stable_decelerating != 0U)
  {
    /*
     * 任务 5 的减速曲线两端一阶、二阶导数均为零。曲线完成后先把四轮指令清零并
     * 进入 TB6612 短路制动，再冻结计时，确保显示时间包含完整的受控停车过程。
     */
    decel_progress = My_gray_s_curve_My(
      My_gray_pid_control_My.stable_decel_step,
      MY_GRAY_STABLE_DECEL_STEPS);
    My_gray_pid_control_My.ramp_pwm =
      My_gray_pid_control_My.stable_decel_start_pwm * (1.0f - decel_progress);
    if (My_gray_pid_control_My.stable_decel_step < MY_GRAY_STABLE_DECEL_STEPS)
    {
      My_gray_pid_control_My.stable_decel_step++;
    }
    else
    {
      My_gray_pid_stop_My();
      My_timer_stop_My();
      return;
    }
  }
  else
  {
    /*
     * 任务 2 前 13 秒保持较高基础速度，之后切换到低速段；任务 5 的基础速度从
     * 启动即等于该低速值，因此不会发生中途速度阶跃，利于钢球稳定。
     */
    elapsed_ms = My_timer_get_elapsed_ms_My();
    if ((elapsed_ms >= MY_GRAY_TASK2_SLOWDOWN_TIME_MS) &&
        (My_gray_pid_control_My.base_pwm > MY_GRAY_TASK2_SLOW_BASE_PWM))
    {
      My_gray_pid_control_My.base_pwm = MY_GRAY_TASK2_SLOW_BASE_PWM;
    }

    My_gray_update_ramp_My();
  }

  if (active_count == 0U)
  {
    if (My_gray_pid_control_My.stable_decelerating != 0U)
    {
      /*
       * 终点宽线离开传感器后仍必须继续降低速度，不能沿用上一周期输出。丢线期间
       * 采用零转向直线减速，避免未知赛道误差造成停车前突然转向。
       */
      My_gray_pid_control_My.line_detected = 0U;
      My_gray_pid_control_My.correction = 0.0f;
      My_move_mecanum_inverse_My(My_gray_pid_control_My.ramp_pwm, 0.0f, 0.0f);
      My_move_velocity_pid_update_My();
      return;
    }
    /*
     * 当前采样未检测到黑线，只更新丢线状态并保持上一周期的误差和四轮速度目标。
     * 速度 PID 仍使用最新编码器反馈逐周期更新 PWM，使车辆沿最后一次有效修正方向
     * 继续运动，直到传感器重新检测到黑线。首次启动前目标为零，因此尚无有效
     * 循迹目标时进入本分支只会维持零速闭环。
     *
     * 此策略不设置丢线超时：传感器持续故障时车辆会持续执行最后指令，实车
     * 使用时必须依靠人工急停或上层任务状态机终止运动。
     */
    My_gray_pid_control_My.line_detected = 0U;
    My_move_velocity_pid_update_My();
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
