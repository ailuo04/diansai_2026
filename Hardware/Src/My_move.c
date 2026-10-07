#include "My_move.h"
#include "tim.h"

static volatile int16_t My_move_velocity[MY_MOVE_WHEEL_COUNT + 1U];      /* 四路轮子的单周期编码器增量，下标 1~4 有效。 */
static volatile int32_t My_move_encoder[MY_MOVE_WHEEL_COUNT + 1U];       /* 兼容旧接口保留的累计值；当前速度闭环不更新。 */
static volatile float My_move_target[MY_MOVE_WHEEL_COUNT + 1U];          /* 四路轮子的目标速度或开环 PWM 目标，下标 1~4 有效。 */
static volatile My_pid_t My_move_speed_pid[MY_MOVE_WHEEL_COUNT + 1U]; /* 四路轮子的速度 PID 控制器，下标 1~4 有效。 */
static volatile uint8_t My_move_speed_pid_ready;                            /* 速度 PID 参数初始化标志，0 表示未初始化。 */
static volatile int16_t My_move_wheel2_period_count_My; /* 2 号轮 EXTI 在当前 TIM6 周期内累计的正交边沿增量。 */
static uint8_t My_move_wheel2_last_state_My;            /* 2 号轮 A/B 相上一状态，仅由 EXTI 与复位流程访问。 */
static uint32_t My_move_tim2_last_count_My;              /* 1 号轮 TIM2 上一周期 32 位 CNT 快照。 */
static uint16_t My_move_tim3_last_count_My;              /* 3 号轮 TIM3 上一周期 16 位 CNT 快照。 */
static uint16_t My_move_tim4_last_count_My;              /* 4 号轮 TIM4 上一周期 16 位 CNT 快照。 */

volatile int16_t My_move_debug_velocity_1_My; /* Keil Watch 调试用：1 号轮 10 ms 编码器速度。 */
volatile int16_t My_move_debug_velocity_2_My; /* Keil Watch 调试用：2 号轮 10 ms 编码器速度。 */
volatile int16_t My_move_debug_velocity_3_My; /* Keil Watch 调试用：3 号轮 10 ms 编码器速度。 */
volatile int16_t My_move_debug_velocity_4_My; /* Keil Watch 调试用：4 号轮 10 ms 编码器速度。 */

/* 若实车发现某一路速度正负与电机机械正方向相反，只需把对应宏改为 -1 后重新编译。 */
#define MY_MOVE_ENCODER_1_DIRECTION  1
#define MY_MOVE_ENCODER_2_DIRECTION  1
#define MY_MOVE_ENCODER_3_DIRECTION -1
#define MY_MOVE_ENCODER_4_DIRECTION  1

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
static const My_move_motor_config_t My_move_motor_config[MY_MOVE_WHEEL_COUNT] = /* 四个实际轮号对应的方向引脚和正向电平配置表。 */
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
    /* PWM 已在改向前拉低；TB6612 两路输出均为低电平，电机处于短路制动状态。 */
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

/**
  * @brief 将 32 位编码器周期增量压入速度环使用的 int16_t 范围。
  * @param value 原始周期增量
  * @retval 限幅后的周期增量
  */
static int16_t My_move_limit_encoder_delta_My(int32_t value)
{
  if (value > 32767L)
  {
    return 32767;
  }
  if (value < -32768L)
  {
    return (int16_t)-32768;
  }

  return (int16_t)value;
}

/**
  * @brief 读取 32 位硬件编码器相邻周期的有符号差值。
  * @details 使用无符号减法保留 32 位环形回绕，再解释为有符号增量；不清零 CNT，
  *          避免编码器边沿恰好落在“读取后清零”窗口时被漏计。一个 10 ms 周期内
  *          的真实增量绝对值必须小于 2^31。
  * @param htim 32 位硬件编码器定时器句柄
  * @param last_count 上一周期 CNT 快照
  * @retval 本周期有符号编码器增量
  */
static int16_t My_move_read_32bit_encoder_delta_My(TIM_HandleTypeDef *htim,
                                                   uint32_t *last_count)
{
  uint32_t current_count = __HAL_TIM_GET_COUNTER(htim); /* 本周期结束时的 CNT 快照。 */
  uint32_t wrapped_delta = current_count - *last_count; /* 按 32 位模空间得到的增量位模式。 */
  int32_t delta = (int32_t)wrapped_delta;               /* 解释为有符号正反转增量。 */

  *last_count = current_count;
  return My_move_limit_encoder_delta_My(delta);
}

/**
  * @brief 读取 16 位硬件编码器相邻周期的有符号差值。
  * @details TIM3/TIM4 的 ARR 均为 65535；差值先按 uint16_t 环形空间计算，再显式
  *          映射到 -32768~32767。一个 10 ms 周期内的真实增量绝对值必须小于 32768。
  * @param htim 16 位硬件编码器定时器句柄
  * @param last_count 上一周期 CNT 快照
  * @retval 本周期有符号编码器增量
  */
static int16_t My_move_read_16bit_encoder_delta_My(TIM_HandleTypeDef *htim,
                                                   uint16_t *last_count)
{
  uint16_t current_count = (uint16_t)__HAL_TIM_GET_COUNTER(htim); /* 本周期 CNT 快照。 */
  uint16_t wrapped_delta = (uint16_t)(current_count - *last_count); /* 16 位环形差值。 */
  int16_t delta = (wrapped_delta <= 32767U)
                    ? (int16_t)wrapped_delta
                    : (int16_t)((int32_t)wrapped_delta - 65536L);

  *last_count = current_count;
  return delta;
}

/**
  * @brief 读取 2 号轮 A/B 相当前逻辑状态。
  * @retval bit0 为 A 相，bit1 为 B 相
  */
static uint8_t My_move_read_wheel2_state_My(void)
{
  uint8_t state = 0U; /* 两位正交状态快照。 */

  if (HAL_GPIO_ReadPin(Encoder_1A_GPIO_Port, Encoder_1A_Pin) == GPIO_PIN_SET)
  {
    state |= 0x01U;
  }
  if (HAL_GPIO_ReadPin(Encoder_1B_GPIO_Port, Encoder_1B_Pin) == GPIO_PIN_SET)
  {
    state |= 0x02U;
  }

  return state;
}

/**
  * @brief 根据相邻 A/B 状态计算正交编码器一步增量。
  * @param previous_state 上一次 A/B 两位状态
  * @param current_state 当前 A/B 两位状态
  * @retval +1、-1 或 0；0 表示重复状态或两位同时跳变的非法状态
  */
static int8_t My_move_decode_wheel2_delta_My(uint8_t previous_state, uint8_t current_state)
{
  static const int8_t decode_table[16] =
  {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
  }; /* 下标为 previous_state<<2 | current_state，非法跳变按 0 处理。 */

  return decode_table[((previous_state & 0x03U) << 2) | (current_state & 0x03U)];
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
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * 四轮非零 PWM 只允许由速度 PID 产生。这里装载保守起始参数，保证应用层首次
   * 下发速度目标前控制器已经就绪；实车必须根据 10 ms 周期脉冲数重新整定。
   */
  My_move_set_speed_pid_My(MY_MOVE_SPEED_PID_DEFAULT_KP,
                           MY_MOVE_SPEED_PID_DEFAULT_KI,
                           MY_MOVE_SPEED_PID_DEFAULT_KD,
                           0.0f,
                           (float)MY_MOVE_PWM_MAX);
  My_move_stop_My();

  return HAL_OK;
}

/**
  * @brief 将四路速度 PID 结果写入底盘电机方向和 TIM5 PWM。
  * @details 本函数是四轮非零 PWM 的唯一底层出口，只允许由速度 PID 更新函数调用；
  *          安全停机可调用本函数写入全零。调用前每路指令都会再次限幅。
  * @param wheel_1 1 号轮速度 PID 输出
  * @param wheel_2 2 号轮速度 PID 输出
  * @param wheel_3 3 号轮速度 PID 输出
  * @param wheel_4 4 号轮速度 PID 输出
  */
static void My_move_apply_pwm_My(int16_t wheel_1,
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

  /* 安全停止必须立即关断四路输出，不等待速度 PID 逐周期收敛。 */
  My_move_apply_pwm_My(0, 0, 0, 0);
}

void My_move_update_encoder_My(void)
{
  int16_t wheel2_delta; /* 从 EXTI 软件计数器取出的 2 号轮本周期增量。 */
  uint32_t interrupt_mask = __get_PRIMASK(); /* 保存进入临界区前的中断屏蔽状态。 */

  /*
   * 2 号轮计数由 EXTI9_5_IRQHandler 异步累加。TIM6 周期读取并清零必须短暂屏蔽
   * 中断，避免清零瞬间丢失外部边沿；临界区内不访问硬件定时器，保持占用极短。
   */
  __disable_irq();
  wheel2_delta = My_move_wheel2_period_count_My;
  My_move_wheel2_period_count_My = 0;
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }

  /*
   * 三路硬件编码器使用相邻 CNT 快照求 10 ms 周期增量，速度环直接使用本周期
   * 脉冲数。这里不清零硬件 CNT，也不维护长期位置累计，避免丢边沿和无用状态。
   *
   * 实车逐轮验证表明，TIM2 对应物理 1 号轮，PA8/PA9 外部中断对应物理 2 号轮。
   * 这里按物理轮号写入速度数组，确保每个速度 PID 使用本轮反馈而不是相邻轮反馈。
   */
  My_move_velocity[1] = My_move_limit_encoder_delta_My(
    (int32_t)My_move_read_32bit_encoder_delta_My(&htim2, &My_move_tim2_last_count_My) *
    MY_MOVE_ENCODER_1_DIRECTION);
  My_move_velocity[2] = My_move_limit_encoder_delta_My(
    (int32_t)wheel2_delta * MY_MOVE_ENCODER_2_DIRECTION);
  My_move_velocity[3] = My_move_limit_encoder_delta_My(
    (int32_t)My_move_read_16bit_encoder_delta_My(&htim3, &My_move_tim3_last_count_My) *
    MY_MOVE_ENCODER_3_DIRECTION);
  My_move_velocity[4] = My_move_limit_encoder_delta_My(
    (int32_t)My_move_read_16bit_encoder_delta_My(&htim4, &My_move_tim4_last_count_My) *
    MY_MOVE_ENCODER_4_DIRECTION);

  /*
   * 将文件内私有速度数组镜像到全局 volatile 调试变量，便于 Keil Debug Watch 直接
   * 添加符号观察。这里不参与控制计算，也不改变速度环数据源，只反映本周期采样结果。
   */
  My_move_debug_velocity_1_My = My_move_velocity[1];
  My_move_debug_velocity_2_My = My_move_velocity[2];
  My_move_debug_velocity_3_My = My_move_velocity[3];
  My_move_debug_velocity_4_My = My_move_velocity[4];
}

void My_move_reset_encoder_My(void)
{
  uint8_t wheel; /* 当前正在清零软件计数的轮号。 */
  uint32_t interrupt_mask = __get_PRIMASK(); /* 保存复位前中断屏蔽状态，保证 EXTI 计数原子清零。 */

  __disable_irq();
  My_move_wheel2_period_count_My = 0;
  My_move_wheel2_last_state_My = My_move_read_wheel2_state_My();
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  My_move_tim2_last_count_My = 0U;
  My_move_tim3_last_count_My = 0U;
  My_move_tim4_last_count_My = 0U;

  for (wheel = 1U; wheel <= MY_MOVE_WHEEL_COUNT; wheel++)
  {
    My_move_velocity[wheel] = 0;
    My_move_encoder[wheel] = 0;
  }

  /*
   * 复位时同步清零调试镜像，避免 Watch 窗口继续显示复位前的速度值而误判编码器仍在运动。
   */
  My_move_debug_velocity_1_My = 0;
  My_move_debug_velocity_2_My = 0;
  My_move_debug_velocity_3_My = 0;
  My_move_debug_velocity_4_My = 0;

  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief 处理 2 号轮外部中断编码器边沿。
  * @details 中断来源为 PA8/PA9 的 EXTI9_5 共享入口，触发条件为 A/B 相上下沿。
  *          函数只读取当前两相电平、查表判断方向并更新 volatile 周期计数；计数会
  *          在 TIM6 周期函数中被原子取走并清零。中断上下文禁止阻塞、打印、动态
  *          内存分配或复杂速度计算，防止影响控制周期和串口 DMA 中断。
  * @param GPIO_Pin 本次触发外部中断的 GPIO 引脚
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t current_state; /* 本次边沿到来后立即读取的 A/B 状态。 */
  int8_t delta;          /* 由状态转移表得到的单边沿计数方向。 */

  if (GPIO_Pin != Encoder_1A_Pin && GPIO_Pin != Encoder_1B_Pin)
  {
    return;
  }

  current_state = My_move_read_wheel2_state_My();
  delta = My_move_decode_wheel2_delta_My(My_move_wheel2_last_state_My, current_state);
  My_move_wheel2_last_state_My = current_state;

  if (delta != 0)
  {
    My_move_wheel2_period_count_My = My_move_limit_encoder_delta_My(
      (int32_t)My_move_wheel2_period_count_My + (int32_t)delta);
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

  /*
   * PID 参数未就绪属于控制链路故障。此时禁止把速度目标直接作为 PWM，只允许
   * 立即输出全零，避免任何上层接口绕过编码器反馈。
   */
  if (My_move_speed_pid_ready == 0U)
  {
    My_move_apply_pwm_My(0, 0, 0, 0);
    return;
  }

  for (wheel = 1U; wheel <= MY_MOVE_WHEEL_COUNT; wheel++)
  {
    float value = My_pid_calc_incremental_My(&My_move_speed_pid[wheel],
                                             My_move_target[wheel],
                                             (float)My_move_velocity[wheel],
                                             MY_MOVE_DEFAULT_TARGET_LIMIT); /* 当前轮 PID 计算得到的浮点 PWM 输出。 */
    output[wheel] = My_move_float_to_pwm_My(value);
  }

  My_move_apply_pwm_My(output[1], output[2], output[3], output[4]);
}
