#include "My_move.h"
#include "tim.h"

static volatile int16_t My_move_velocity[MY_MOVE_WHEEL_COUNT + 1U];      /* 四路轮子的单周期编码器增量，下标 1~4 有效。 */
static volatile int32_t My_move_encoder[MY_MOVE_WHEEL_COUNT + 1U];       /* 四路轮子的累计编码器计数，下标 1~4 有效。 */
static volatile float My_move_target[MY_MOVE_WHEEL_COUNT + 1U];          /* 四路轮子的目标速度或开环 PWM 目标，下标 1~4 有效。 */
static volatile My_pid_t My_move_speed_pid[MY_MOVE_WHEEL_COUNT + 1U]; /* 四路轮子的速度 PID 控制器，下标 1~4 有效。 */
static volatile uint8_t My_move_speed_pid_ready;                            /* 速度 PID 参数初始化标志，0 表示未初始化。 */

typedef struct
{
  GPIO_TypeDef *gpio_a;            /* TB6612 的 IN1 端口。 */
  uint16_t pin_a;                  /* TB6612 的 IN1 引脚。 */
  GPIO_TypeDef *gpio_b;            /* TB6612 的 IN2 端口。 */
  uint16_t pin_b;                  /* TB6612 的 IN2 引脚。 */
  GPIO_PinState forward_pin_a;     /* 底盘前进时 IN1 的有效电平。 */
} My_move_motor_config_t;

/*
 * 四个电机的接线与机械正方向配置，数组下标 0~3 对应实际电机 1~4。
 * 实车验证表明板上 MOTOR_2 方向通道连接实际 1 号轮，MOTOR_1 方向通道连接
 * 实际 2 号轮，因此前两项按实际轮号交换；3、4 号轮保持 CubeMX 标号映射。
 */
static const My_move_motor_config_t My_move_motor_config[MY_MOVE_WHEEL_COUNT] =
{
  {MOTOR_2A_GPIO_Port, MOTOR_2A_Pin, MOTOR_2B_GPIO_Port, MOTOR_2B_Pin, GPIO_PIN_RESET},
  {MOTOR_1A_GPIO_Port, MOTOR_1A_Pin, MOTOR_1B_GPIO_Port, MOTOR_1B_Pin, GPIO_PIN_RESET},
  {MOTOR_3A_GPIO_Port, MOTOR_3A_Pin, MOTOR_3B_GPIO_Port, MOTOR_3B_Pin, GPIO_PIN_SET},
  {MOTOR_4A_GPIO_Port, MOTOR_4A_Pin, MOTOR_4B_GPIO_Port, MOTOR_4B_Pin, GPIO_PIN_SET}
};

/**
  * @brief 判断轮号是否合法。
  * @param wheel 轮号
  * @retval 1 表示合法，0 表示非法
  */
static uint8_t My_move_is_valid_wheel_My(uint8_t wheel)
{
  return (uint8_t)(wheel >= 1U && wheel <= MY_MOVE_WHEEL_COUNT);
}

/**
  * @brief 将电机指令限制在 PWM 安全范围内。
  * @param value 原始指令
  * @retval 限幅后的指令
  */
static int16_t My_move_limit_pwm_My(int32_t value)
{
  if (value > MY_MOVE_PWM_MAX)
  {
    return MY_MOVE_PWM_MAX;
  }
  if (value < -MY_MOVE_PWM_MAX)
  {
    return -MY_MOVE_PWM_MAX;
  }

  return (int16_t)value;
}

/**
  * @brief 写入指定 PWM 通道占空比。
  * @param wheel 轮号，范围 1~4
  * @param duty 占空比
  */
static void My_move_set_pwm_My(uint8_t wheel, uint16_t duty)
{
  switch (wheel)
  {
    case 1U:
      /* 板上 TIM5_CH2 实际连接 1 号轮，与方向映射的 MOTOR_2 通道配套使用。 */
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, duty);
      break;
    case 2U:
      /* 板上 TIM5_CH1 实际连接 2 号轮，与方向映射的 MOTOR_1 通道配套使用。 */
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, duty);
      break;
    case 3U:
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, duty);
      break;
    case 4U:
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, duty);
      break;
    default:
      break;
  }
}

/**
  * @brief 按底盘机械正方向设置单个 TB6612 电机方向。
  * @param wheel 轮号，范围 1~4
  * @param command 电机指令，正负表示方向
  */
static void My_move_set_direction_My(uint8_t wheel, int16_t command)
{
  const My_move_motor_config_t *config; /* 当前电机的方向引脚与正转极性配置。 */
  GPIO_PinState pin_a; /* 当前轮 A 相方向引脚电平。 */
  GPIO_PinState pin_b; /* 当前轮 B 相方向引脚电平。 */

  if (My_move_is_valid_wheel_My(wheel) == 0U)
  {
    return;
  }

  config = &My_move_motor_config[wheel - 1U];

  if (command == 0)
  {
    /* TB6612 的 IN1=0、IN2=0 为滑行停止。 */
    pin_a = GPIO_PIN_RESET;
    pin_b = GPIO_PIN_RESET;
  }
  else
  {
    pin_a = (command > 0) ? config->forward_pin_a : (GPIO_PinState)!config->forward_pin_a;
    pin_b = (GPIO_PinState)!pin_a;
  }

  HAL_GPIO_WritePin(config->gpio_a, config->pin_a, pin_a);
  HAL_GPIO_WritePin(config->gpio_b, config->pin_b, pin_b);
}

/**
  * @brief 将浮点输出转换为 PWM 指令。
  * @param value 浮点输出
  * @retval 限幅后的整数 PWM 指令
  */
static int16_t My_move_float_to_pwm_My(float value)
{
  if (value >= 0.0f)
  {
    return My_move_limit_pwm_My((int32_t)(value + 0.5f));
  }

  return My_move_limit_pwm_My((int32_t)(value - 0.5f));
}

HAL_StatusTypeDef My_move_init_My(void)
{
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK)
  {
    return HAL_ERROR;
  }

  My_move_reset_encoder_My();
  My_move_stop_My();

  return HAL_OK;
}

void My_move_control_My(int16_t wheel_1,
                              int16_t wheel_2,
                              int16_t wheel_3,
                              int16_t wheel_4)
{
  int16_t command[MY_MOVE_WHEEL_COUNT + 1U]; /* 四路限幅后的电机指令，下标 1~4 有效。 */
  uint8_t wheel;                                /* 当前正在处理的轮号。 */

  command[1] = My_move_limit_pwm_My(wheel_1);
  command[2] = My_move_limit_pwm_My(wheel_2);
  command[3] = My_move_limit_pwm_My(wheel_3);
  command[4] = My_move_limit_pwm_My(wheel_4);

  for (wheel = 1U; wheel <= MY_MOVE_WHEEL_COUNT; wheel++)
  {
    uint16_t duty = (command[wheel] >= 0) ? (uint16_t)command[wheel] : (uint16_t)(-command[wheel]); /* 写入 PWM 的占空比绝对值。 */

    /* 先关闭当前通道输出，再改变 TB6612 方向输入，避免带载直接换向。 */
    My_move_set_pwm_My(wheel, 0U);
    My_move_set_direction_My(wheel, command[wheel]);
    My_move_set_pwm_My(wheel, duty);
  }
}

void My_move_stop_My(void)
{
  uint8_t wheel; /* 当前正在复位目标和 PID 状态的轮号。 */

  for (wheel = 1U; wheel <= MY_MOVE_WHEEL_COUNT; wheel++)
  {
    My_move_target[wheel] = 0.0f;
    My_pid_reset_My(&My_move_speed_pid[wheel]);
  }

  My_move_control_My(0, 0, 0, 0);
}

void My_move_update_encoder_My(void)
{
  /* 当前未接入电机编码器，保持速度反馈为零，运动控制采用开环 PWM。 */
  My_move_velocity[1] = 0;
  My_move_velocity[2] = 0;
  My_move_velocity[3] = 0;
  My_move_velocity[4] = 0;
}

void My_move_reset_encoder_My(void)
{
  uint8_t wheel; /* 当前正在清零软件计数的轮号。 */

  for (wheel = 1U; wheel <= MY_MOVE_WHEEL_COUNT; wheel++)
  {
    My_move_velocity[wheel] = 0;
    My_move_encoder[wheel] = 0;
  }
}

int16_t My_move_get_velocity_My(uint8_t wheel)
{
  if (My_move_is_valid_wheel_My(wheel) == 0U)
  {
    return 0;
  }

  return My_move_velocity[wheel];
}

int32_t My_move_get_encoder_My(uint8_t wheel)
{
  if (My_move_is_valid_wheel_My(wheel) == 0U)
  {
    return 0;
  }

  return My_move_encoder[wheel];
}

void My_move_set_speed_pid_My(float kp,
                                    float ki,
                                    float kd,
                                    float integral_limit,
                                    float output_limit)
{
  uint8_t wheel; /* 当前正在写入 PID 参数的轮号。 */

  for (wheel = 1U; wheel <= MY_MOVE_WHEEL_COUNT; wheel++)
  {
    My_pid_init_My(&My_move_speed_pid[wheel], kp, ki, kd, integral_limit, output_limit);
  }
  My_move_speed_pid_ready = 1U;
}

void My_move_set_target_My(float wheel_1,
                                 float wheel_2,
                                 float wheel_3,
                                 float wheel_4)
{
  My_move_target[1] = wheel_1;
  My_move_target[2] = wheel_2;
  My_move_target[3] = wheel_3;
  My_move_target[4] = wheel_4;
}

/**
  * @brief 按实际轮位执行麦克纳姆轮逆运动学解算。
  * @param move_vx 车体前后方向速度，正值表示沿机械正方向前进
  * @param move_vy 车体横向速度
  * @param move_vw 车体旋转修正量，正值使左侧轮减速、右侧轮加速
  * @note 轮位固定为1号左上、2号右上、3号右下、4号左下。
  */
void My_move_mecanum_inverse_My(float move_vx, float move_vy, float move_vw)
{
  /* 按实际顺时针轮号排列目标值，确保纯旋转时左右两侧形成相反差速。 */
  My_move_set_target_My(move_vx - move_vy - move_vw,
                        move_vx + move_vy + move_vw,
                        move_vx - move_vy + move_vw,
                        move_vx + move_vy - move_vw);
}

void My_move_velocity_pid_update_My(void)
{
  int16_t output[MY_MOVE_WHEEL_COUNT + 1U]; /* 四路速度闭环计算后的 PWM 输出，下标 1~4 有效。 */
  uint8_t wheel;                               /* 当前正在计算速度闭环的轮号。 */

  My_move_update_encoder_My();

  for (wheel = 1U; wheel <= MY_MOVE_WHEEL_COUNT; wheel++)
  {
    if (My_move_speed_pid_ready != 0U)
    {
      float value = My_pid_calc_incremental_My(&My_move_speed_pid[wheel],
                                                     My_move_target[wheel],
                                                     (float)My_move_velocity[wheel],
                                                     MY_MOVE_DEFAULT_TARGET_LIMIT); /* 当前轮 PID 计算得到的浮点 PWM 输出。 */
      output[wheel] = My_move_float_to_pwm_My(value);
    }
    else
    {
      output[wheel] = My_move_float_to_pwm_My(My_move_target[wheel]);
    }
  }

  My_move_control_My(output[1], output[2], output[3], output[4]);
}
