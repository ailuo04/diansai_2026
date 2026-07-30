#include "My_steering.h"

#include "My_encoder.h"
#include "main.h"
#include "tim.h"

volatile My_steering_control_t My_steering_control_My;

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
  * @brief 向转向电机驱动器写入方向和 PWM。
  * @param command 有符号 PWM，正负表示逻辑方向
  * @details 每次先把 TIM4_CH3 比较值清零，再切换 Steering_1A/1B，防止带载
  *          直接换向。默认正方向为 1A=1、1B=0；motor_direction 可整体反转。
  */
static void My_steering_write_output_My(int16_t command)
{
  int32_t mapped_command = (int32_t)command * (int32_t)My_steering_control_My.motor_direction;
  uint16_t duty;

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
  My_steering_control_My.actual_count = My_encoder_debug_My.total_count;
  My_steering_control_My.target_count = My_steering_control_My.actual_count;
  My_steering_control_My.pwm_output = 0;

  /* 默认参数仅建立保守的计数位置 P 环；实车使用前应根据负载和编码器分辨率整定。 */
  My_pid_init_My(&My_steering_control_My.pid, 1.0f, 0.0f, 0.0f, 1000.0f, 300.0f);
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
  uint32_t interrupt_mask = __get_PRIMASK();

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

void My_steering_start_My(int32_t target_count)
{
  uint32_t interrupt_mask = __get_PRIMASK();

  /* 启动状态与 PID 历史由 TIM6 中断共同使用，需原子完成整个状态切换。 */
  __disable_irq();
  My_pid_reset_My(&My_steering_control_My.pid);
  My_steering_control_My.target_count = target_count;
  My_steering_control_My.enabled = 1U;
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

void My_steering_set_target_My(int32_t target_count)
{
  /* Cortex-M4 对自然对齐的 32 位变量单次读写具有原子性。 */
  My_steering_control_My.target_count = target_count;
}

void My_steering_stop_My(void)
{
  uint32_t interrupt_mask = __get_PRIMASK();

  /* 先禁止周期输出，再锁定当前位置并清除 PID，确保退出临界区后不会恢复旧指令。 */
  __disable_irq();
  My_steering_control_My.enabled = 0U;
  My_steering_control_My.actual_count = My_encoder_debug_My.total_count;
  My_steering_control_My.target_count = My_steering_control_My.actual_count;
  My_steering_control_My.pwm_output = 0;
  My_pid_reset_My(&My_steering_control_My.pid);
  My_steering_write_output_My(0);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

void My_steering_update_10ms_My(void)
{
  float pid_output;

  /* 编码器已在本次 TIM6 中断前半段采样，此处读取同一周期的软件累计位置。 */
  My_steering_control_My.actual_count = My_encoder_debug_My.total_count;

  if (My_steering_control_My.enabled == 0U)
  {
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
