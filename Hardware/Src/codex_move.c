#include "codex_move.h"
#include "tim.h"

static volatile int16_t codex_move_velocity[CODEX_MOVE_WHEEL_COUNT + 1U];
static volatile int32_t codex_move_encoder[CODEX_MOVE_WHEEL_COUNT + 1U];
static volatile float codex_move_target[CODEX_MOVE_WHEEL_COUNT + 1U];
static volatile codex_pid_t codex_move_speed_pid[CODEX_MOVE_WHEEL_COUNT + 1U];
static volatile uint8_t codex_move_speed_pid_ready;

/**
  * @brief 判断轮号是否合法。
  * @param wheel 轮号
  * @retval 1 表示合法，0 表示非法
  */
static uint8_t codex_move_is_valid_wheel_codex(uint8_t wheel)
{
  return (uint8_t)(wheel >= 1U && wheel <= CODEX_MOVE_WHEEL_COUNT);
}

/**
  * @brief 将电机指令限制在 PWM 安全范围内。
  * @param value 原始指令
  * @retval 限幅后的指令
  */
static int16_t codex_move_limit_pwm_codex(int32_t value)
{
  if (value > CODEX_MOVE_PWM_MAX)
  {
    return CODEX_MOVE_PWM_MAX;
  }
  if (value < -CODEX_MOVE_PWM_MAX)
  {
    return -CODEX_MOVE_PWM_MAX;
  }

  return (int16_t)value;
}

/**
  * @brief 写入指定 PWM 通道占空比。
  * @param wheel 轮号，范围 1~4
  * @param duty 占空比
  */
static void codex_move_set_pwm_codex(uint8_t wheel, uint16_t duty)
{
  switch (wheel)
  {
    case 1U:
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty);
      break;
    case 2U:
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty);
      break;
    case 3U:
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty);
      break;
    case 4U:
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, duty);
      break;
    default:
      break;
  }
}

/**
  * @brief 按旧工程方向极性设置单个电机方向。
  * @param wheel 轮号，范围 1~4
  * @param command 电机指令，正负表示方向
  */
static void codex_move_set_direction_codex(uint8_t wheel, int16_t command)
{
  GPIO_PinState pin_a;
  GPIO_PinState pin_b;

  if (command == 0)
  {
    pin_a = GPIO_PIN_RESET;
    pin_b = GPIO_PIN_RESET;
  }
  else if (wheel == 1U || wheel == 4U)
  {
    pin_a = (command > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    pin_b = (command > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }
  else
  {
    pin_a = (command > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    pin_b = (command > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  switch (wheel)
  {
    case 1U:
      HAL_GPIO_WritePin(Move_1A_GPIO_Port, Move_1A_Pin, pin_a);
      HAL_GPIO_WritePin(Move_1B_GPIO_Port, Move_1B_Pin, pin_b);
      break;
    case 2U:
      HAL_GPIO_WritePin(Move_2A_GPIO_Port, Move_2A_Pin, pin_a);
      HAL_GPIO_WritePin(Move_2B_GPIO_Port, Move_2B_Pin, pin_b);
      break;
    case 3U:
      HAL_GPIO_WritePin(Move_3A_GPIO_Port, Move_3A_Pin, pin_a);
      HAL_GPIO_WritePin(Move_3B_GPIO_Port, Move_3B_Pin, pin_b);
      break;
    case 4U:
      HAL_GPIO_WritePin(Move_4A_GPIO_Port, Move_4A_Pin, pin_a);
      HAL_GPIO_WritePin(Move_4B_GPIO_Port, Move_4B_Pin, pin_b);
      break;
    default:
      break;
  }
}

/**
  * @brief 读取一个编码器周期增量并清零计数器。
  * @param htim 编码器定时器句柄
  * @retval 本周期有符号增量
  */
static int16_t codex_move_read_encoder_delta_codex(TIM_HandleTypeDef *htim)
{
  int16_t delta;

  delta = (int16_t)__HAL_TIM_GET_COUNTER(htim);
  __HAL_TIM_SET_COUNTER(htim, 0U);

  return delta;
}

/**
  * @brief 将浮点输出转换为 PWM 指令。
  * @param value 浮点输出
  * @retval 限幅后的整数 PWM 指令
  */
static int16_t codex_move_float_to_pwm_codex(float value)
{
  if (value >= 0.0f)
  {
    return codex_move_limit_pwm_codex((int32_t)(value + 0.5f));
  }

  return codex_move_limit_pwm_codex((int32_t)(value - 0.5f));
}

HAL_StatusTypeDef codex_move_init_codex(void)
{
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK)
  {
    return HAL_ERROR;
  }

  codex_move_reset_encoder_codex();
  codex_move_stop_codex();

  return HAL_OK;
}

void codex_move_control_codex(int16_t wheel_1,
                              int16_t wheel_2,
                              int16_t wheel_3,
                              int16_t wheel_4)
{
  int16_t command[CODEX_MOVE_WHEEL_COUNT + 1U];
  uint8_t wheel;

  command[1] = codex_move_limit_pwm_codex(wheel_1);
  command[2] = codex_move_limit_pwm_codex(wheel_2);
  command[3] = codex_move_limit_pwm_codex(wheel_3);
  command[4] = codex_move_limit_pwm_codex(wheel_4);

  for (wheel = 1U; wheel <= CODEX_MOVE_WHEEL_COUNT; wheel++)
  {
    uint16_t duty = (command[wheel] >= 0) ? (uint16_t)command[wheel] : (uint16_t)(-command[wheel]);
    codex_move_set_direction_codex(wheel, command[wheel]);
    codex_move_set_pwm_codex(wheel, duty);
  }
}

void codex_move_stop_codex(void)
{
  uint8_t wheel;

  for (wheel = 1U; wheel <= CODEX_MOVE_WHEEL_COUNT; wheel++)
  {
    codex_move_target[wheel] = 0.0f;
    codex_pid_reset_codex(&codex_move_speed_pid[wheel]);
  }

  codex_move_control_codex(0, 0, 0, 0);
}

void codex_move_update_encoder_codex(void)
{
  codex_move_velocity[1] = codex_move_read_encoder_delta_codex(&htim3);
  codex_move_velocity[2] = codex_move_read_encoder_delta_codex(&htim2);
  codex_move_velocity[3] = codex_move_read_encoder_delta_codex(&htim1);
  codex_move_velocity[4] = codex_move_read_encoder_delta_codex(&htim4);

  codex_move_encoder[1] += codex_move_velocity[1];
  codex_move_encoder[2] += codex_move_velocity[2];
  codex_move_encoder[3] += codex_move_velocity[3];
  codex_move_encoder[4] += codex_move_velocity[4];
}

void codex_move_reset_encoder_codex(void)
{
  uint8_t wheel;

  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  for (wheel = 1U; wheel <= CODEX_MOVE_WHEEL_COUNT; wheel++)
  {
    codex_move_velocity[wheel] = 0;
    codex_move_encoder[wheel] = 0;
  }
}

int16_t codex_move_get_velocity_codex(uint8_t wheel)
{
  if (codex_move_is_valid_wheel_codex(wheel) == 0U)
  {
    return 0;
  }

  return codex_move_velocity[wheel];
}

int32_t codex_move_get_encoder_codex(uint8_t wheel)
{
  if (codex_move_is_valid_wheel_codex(wheel) == 0U)
  {
    return 0;
  }

  return codex_move_encoder[wheel];
}

void codex_move_set_speed_pid_codex(float kp,
                                    float ki,
                                    float kd,
                                    float integral_limit,
                                    float output_limit)
{
  uint8_t wheel;

  for (wheel = 1U; wheel <= CODEX_MOVE_WHEEL_COUNT; wheel++)
  {
    codex_pid_init_codex(&codex_move_speed_pid[wheel], kp, ki, kd, integral_limit, output_limit);
  }
  codex_move_speed_pid_ready = 1U;
}

void codex_move_set_target_codex(float wheel_1,
                                 float wheel_2,
                                 float wheel_3,
                                 float wheel_4)
{
  codex_move_target[1] = wheel_1;
  codex_move_target[2] = wheel_2;
  codex_move_target[3] = wheel_3;
  codex_move_target[4] = wheel_4;
}

void codex_move_mecanum_inverse_codex(float move_vx, float move_vy, float move_vw)
{
  codex_move_set_target_codex(move_vx - move_vy - move_vw,
                              move_vx + move_vy - move_vw,
                              move_vx - move_vy + move_vw,
                              move_vx + move_vy + move_vw);
}

void codex_move_velocity_pid_update_codex(void)
{
  int16_t output[CODEX_MOVE_WHEEL_COUNT + 1U];
  uint8_t wheel;

  codex_move_update_encoder_codex();

  for (wheel = 1U; wheel <= CODEX_MOVE_WHEEL_COUNT; wheel++)
  {
    if (codex_move_speed_pid_ready != 0U)
    {
      float value = codex_pid_calc_incremental_codex(&codex_move_speed_pid[wheel],
                                                     codex_move_target[wheel],
                                                     (float)codex_move_velocity[wheel],
                                                     CODEX_MOVE_DEFAULT_TARGET_LIMIT);
      output[wheel] = codex_move_float_to_pwm_codex(value);
    }
    else
    {
      output[wheel] = codex_move_float_to_pwm_codex(codex_move_target[wheel]);
    }
  }

  codex_move_control_codex(output[1], output[2], output[3], output[4]);
}
