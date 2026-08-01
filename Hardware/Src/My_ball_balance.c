#include "My_ball_balance.h"

#include "My_steering.h"
#include "stm32f4xx_hal.h"

#include <string.h>

#define MY_BALL_POSITION_ABS_LIMIT_MM 125 /* 摄像头协议定义的平台中心到左右端点最大距离。 */
#define MY_BALL_POSITION_SPEED_INTERVAL_MS 20U /* 至少累计 10 ms 位置变化再求速度，降低毫米量化噪声。 */

volatile My_ball_balance_debug_t My_ball_balance_debug_My; /* 供 Keil Watch 观察的摄像头和外环状态。 */

static uint8_t My_ball_frame_buffer_My[MY_BALL_FRAME_LENGTH]; /* 主循环流式重组中的候选固定长度帧。 */
static uint8_t My_ball_parser_index_My; /* 候选帧已缓存字节数，仅由主循环访问。 */
static uint32_t My_ball_speed_last_frame_count_My; /* 速度估算器已经观察过的合法帧计数。 */
static uint32_t My_ball_speed_reference_ms_My; /* 位置差分速度参考点对应的毫秒时刻。 */
static int16_t My_ball_speed_reference_position_mm_My; /* 位置差分速度参考点，单位毫米。 */
static uint8_t My_ball_speed_reference_ready_My; /* 非零表示位置差分参考点已经建立。 */
static uint32_t My_ball_final_arrival_last_frame_count_My; /* 最终到点确认已经检查过的合法帧序号。 */

/**
  * @brief 判断字节是否为十进制数字字符。
  * @param value 待检查字节
  * @retval 非零表示字符位于 '0'～'9'
  */
static uint8_t My_ball_is_digit_My(uint8_t value)
{
  return (uint8_t)(value >= (uint8_t)'0' && value <= (uint8_t)'9');
}

/**
  * @brief 解析固定宽度的无符号十进制字段。
  * @param data 数字字段首地址
  * @param length 字段位数
  * @retval 字段对应的整数值；调用前必须完成数字字符校验
  */
static uint16_t My_ball_parse_unsigned_My(const uint8_t *data, uint8_t length)
{
  uint16_t value = 0U; /* 按十进制从高位到低位累计的字段值。 */
  uint8_t index;       /* 当前解析的数字字符索引。 */

  for (index = 0U; index < length; index++)
  {
    value = (uint16_t)(value * 10U + (uint16_t)(data[index] - (uint8_t)'0'));
  }

  return value;
}

/**
  * @brief 校验并解析一帧固定 13 字节摄像头数据。
  * @param frame 候选帧首地址
  * @param position_mm 解析成功后写入有符号位置
  * @param speed_mm_s 解析成功后写入有符号速度
  * @retval 非零表示帧格式与位置范围均合法
  */
static uint8_t My_ball_parse_frame_My(const uint8_t *frame,
                                      int16_t *position_mm,
                                      int16_t *speed_mm_s)
{
  uint16_t position_abs; /* 位置字段三位数字对应的绝对值。 */
  uint16_t speed_abs;    /* 速度字段四位数字对应的绝对值。 */
  uint8_t index;         /* 当前检查的数字字符索引。 */

  if (frame[0] != (uint8_t)'l' || frame[1] != (uint8_t)'y' ||
      frame[11] != (uint8_t)'c' || frame[12] != (uint8_t)'s' ||
      (frame[2] != (uint8_t)'+' && frame[2] != (uint8_t)'-') ||
      (frame[6] != (uint8_t)'+' && frame[6] != (uint8_t)'-'))
  {
    return 0U;
  }

  for (index = 3U; index <= 5U; index++)
  {
    if (My_ball_is_digit_My(frame[index]) == 0U)
    {
      return 0U;
    }
  }
  for (index = 7U; index <= 10U; index++)
  {
    if (My_ball_is_digit_My(frame[index]) == 0U)
    {
      return 0U;
    }
  }

  position_abs = My_ball_parse_unsigned_My(&frame[3], 3U);
  speed_abs = My_ball_parse_unsigned_My(&frame[7], 4U);
  if (position_abs > MY_BALL_POSITION_ABS_LIMIT_MM)
  {
    return 0U;
  }

  *position_mm = (frame[2] == (uint8_t)'-') ? -(int16_t)position_abs : (int16_t)position_abs;
  *speed_mm_s = (frame[6] == (uint8_t)'-') ? -(int16_t)speed_abs : (int16_t)speed_abs;
  return 1U;
}

/**
  * @brief 在候选帧校验失败后保留其中可能存在的新帧头后缀。
  * @details 从第二个字节开始寻找 `ly`，避免一个坏帧吞掉紧随其后的合法帧头；
  *          若只剩末尾字符 `l`，保留该字符等待下一字节补成帧头。
  */
static void My_ball_resync_My(void)
{
  uint8_t index; /* 当前搜索的候选新帧头起始位置。 */

  for (index = 1U; index < (MY_BALL_FRAME_LENGTH - 1U); index++)
  {
    if (My_ball_frame_buffer_My[index] == (uint8_t)'l' &&
        My_ball_frame_buffer_My[index + 1U] == (uint8_t)'y')
    {
      My_ball_parser_index_My = (uint8_t)(MY_BALL_FRAME_LENGTH - index);
      memmove(My_ball_frame_buffer_My,
              &My_ball_frame_buffer_My[index],
              My_ball_parser_index_My);
      return;
    }
  }

  if (My_ball_frame_buffer_My[MY_BALL_FRAME_LENGTH - 1U] == (uint8_t)'l')
  {
    My_ball_frame_buffer_My[0] = (uint8_t)'l';
    My_ball_parser_index_My = 1U;
  }
  else
  {
    My_ball_parser_index_My = 0U;
  }
}

/**
  * @brief 将浮点值限制在正负对称范围内。
  * @param value 待限幅值
  * @param limit 正的限幅绝对值
  * @retval 限幅后的值
  */
static float My_ball_limit_symmetric_My(float value, float limit)
{
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
  * @brief 计算浮点绝对值。
  * @param value 输入值
  * @retval 输入值的绝对值
  */
static float My_ball_abs_My(float value)
{
  return (value < 0.0f) ? -value : value;
}

/**
  * @brief 返回浮点值的方向符号。
  * @param value 待判断方向的数值
  * @retval 正数返回 1，负数返回 -1，零返回 0
  */
static float My_ball_sign_My(float value)
{
  if (value > 0.0f)
  {
    return 1.0f;
  }
  if (value < 0.0f)
  {
    return -1.0f;
  }

  return 0.0f;
}

/**
  * @brief 按固定步长把平台修正角平滑收敛到水平零修正。
  * @param correction_angle_deg 当前已经下发的平台修正角，单位度
  * @retval 本周期允许下发的新修正角，单位度
  * @details 仅用于最终目标到点窗口，避免制动角直接跳回水平角引起平台快速反向；
  *          本函数运行在 TIM6 中断路径，只执行定长比较和加减运算。
  */
static float My_ball_step_correction_to_level_My(float correction_angle_deg)
{
  if (correction_angle_deg > MY_BALL_FINAL_LEVEL_STEP_DEG)
  {
    return correction_angle_deg - MY_BALL_FINAL_LEVEL_STEP_DEG;
  }
  if (correction_angle_deg < -MY_BALL_FINAL_LEVEL_STEP_DEG)
  {
    return correction_angle_deg + MY_BALL_FINAL_LEVEL_STEP_DEG;
  }

  return 0.0f;
}

/**
  * @brief 用相邻摄像头位置和到达时间计算控制所需的钢球速度。
  * @retval 本周期用于驱动和制动判断的速度，单位毫米每秒
  * @details 摄像头帧内速度在实测运动过程中可能长期为零，因此控制器不能把它作为
  *          唯一制动依据。位置差分至少累计 50 ms 后再更新，避免 1 mm 量化在高帧率
  *          下被放大成尖峰；首个有效差分窗口建立前暂用协议速度作为启动过渡。
  */
static float My_ball_update_measured_speed_My(void)
{
  uint32_t frame_count = My_ball_balance_debug_My.valid_frame_count; /* 当前最新合法帧序号。 */

  if (frame_count != My_ball_speed_last_frame_count_My)
  {
    uint32_t update_ms = My_ball_balance_debug_My.last_update_ms; /* 最新位置对应的到达时刻。 */
    int16_t position_mm = My_ball_balance_debug_My.position_mm; /* 最新钢球位置快照。 */

    My_ball_speed_last_frame_count_My = frame_count;
    if (My_ball_speed_reference_ready_My == 0U)
    {
      My_ball_speed_reference_position_mm_My = position_mm;
      My_ball_speed_reference_ms_My = update_ms;
      My_ball_speed_reference_ready_My = 1U;
    }
    else
    {
      uint32_t interval_ms = update_ms - My_ball_speed_reference_ms_My; /* 当前差分窗口时长。 */

      if (interval_ms >= MY_BALL_POSITION_SPEED_INTERVAL_MS &&
          interval_ms <= MY_BALL_COMMUNICATION_TIMEOUT_MS)
      {
        int32_t position_delta_mm = (int32_t)position_mm -
          (int32_t)My_ball_speed_reference_position_mm_My; /* 差分窗口内的有符号位移。 */
        float position_speed = (float)(position_delta_mm * 1000L) /
          (float)interval_ms; /* 位移除以实际到达时间，避免假定摄像头固定帧率。 */

        My_ball_balance_debug_My.position_speed_mm_s = My_ball_limit_symmetric_My(
          position_speed,
          MY_BALL_SPEED_ABS_LIMIT_MM_S);
        My_ball_balance_debug_My.position_speed_ready = 1U;
        My_ball_speed_reference_position_mm_My = position_mm;
        My_ball_speed_reference_ms_My = update_ms;
      }
      else if (interval_ms > MY_BALL_COMMUNICATION_TIMEOUT_MS)
      {
        /* 帧间隔已经失去连续运动意义，重新建立参考点，禁止把长时间位移误算成瞬时速度。 */
        My_ball_speed_reference_position_mm_My = position_mm;
        My_ball_speed_reference_ms_My = update_ms;
        My_ball_balance_debug_My.position_speed_mm_s = 0.0f;
        My_ball_balance_debug_My.position_speed_ready = 0U;
      }
    }
  }

  if (My_ball_balance_debug_My.position_speed_ready != 0U)
  {
    return My_ball_balance_debug_My.position_speed_mm_s;
  }

  return My_ball_limit_symmetric_My(
    (float)My_ball_balance_debug_My.speed_mm_s,
    MY_BALL_SPEED_ABS_LIMIT_MM_S);
}

/**
  * @brief 按距离和速度选择保持、驱动或制动动作。
  * @param position_error_mm 当前目标位置减测量位置，正负表示目标方向
  * @param measured_speed_mm_s 钢球测量速度，正负表示实际运动方向
  * @param drive_angle_deg 当前目标阶段朝目标方向施加的驱动角
  * @param brake_angle_deg 当前目标阶段施加到速度反方向的制动角
  * @param max_speed_mm_s 当前目标阶段允许的最大运动速度
  * @param brake_distance_mm 当前目标阶段开始反向制动的剩余距离
  * @retval 相对水平角的修正量，单位度
  * @details 到点且低速时回到水平；钢球运动方向错误时直接制动；朝目标运动时，
  *          速度达到上限或距离进入制动区也立即制动；其余情况只给固定小驱动角。
  *          本函数由 TIM6 中断路径调用，只执行定长比较和浮点运算，不阻塞、不分配内存。
  */
static float My_ball_select_correction_My(float position_error_mm,
                                          float measured_speed_mm_s,
                                          float drive_angle_deg,
                                          float brake_angle_deg,
                                          float max_speed_mm_s,
                                          float brake_distance_mm)
{
  float distance_abs = My_ball_abs_My(position_error_mm); /* 当前距离目标的绝对距离。 */
  float speed_abs = My_ball_abs_My(measured_speed_mm_s); /* 当前钢球速度绝对值。 */
  uint8_t moving_toward_target =
    (position_error_mm * measured_speed_mm_s > 0.0f) ? 1U : 0U; /* 误差与速度同号表示正在接近目标。 */

  if (distance_abs <= MY_BALL_ARRIVE_DISTANCE_MM &&
      speed_abs <= MY_BALL_ARRIVE_SPEED_MM_S)
  {
    /* 位置和速度同时满足到点条件才回水平，避免高速掠过目标时误判完成。 */
    My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
    return 0.0f;
  }

  if (speed_abs > 0.0f &&
      (moving_toward_target == 0U ||
       speed_abs >= max_speed_mm_s ||
       distance_abs <= brake_distance_mm))
  {
    /*
     * 制动角始终指向当前速度的反方向。运动方向错误时无需等待阈值；朝目标运动时，
     * 速度超限或进入近距离区间任一条件成立就立即反向，防止继续加速冲过目标。
     */
    My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_BRAKE;
    return MY_BALL_CONTROL_DIRECTION *
      (-My_ball_sign_My(measured_speed_mm_s) * brake_angle_deg);
  }

  /* 尚未到点且不需要制动时，只施加固定小角度，不再使用多层比例、积分或卡滞参数。 */
  My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_DRIVE;
  return MY_BALL_CONTROL_DIRECTION *
    (My_ball_sign_My(position_error_mm) * drive_angle_deg);
}

/**
  * @brief 初始化摄像头帧解析、位置差分测速和钢球控制状态。
  * @details 本函数在 TIM6 中断启动前调用，只清零内存状态，不访问阻塞外设。
  */
void My_ball_balance_init_My(void)
{
  My_ball_parser_index_My = 0U;
  memset(My_ball_frame_buffer_My, 0, sizeof(My_ball_frame_buffer_My));

  My_ball_balance_debug_My.position_mm = 0;
  My_ball_balance_debug_My.speed_mm_s = 0;
  My_ball_balance_debug_My.last_update_ms = 0U;
  My_ball_balance_debug_My.valid_frame_count = 0U;
  My_ball_balance_debug_My.invalid_frame_count = 0U;
  My_ball_balance_debug_My.parser_index = 0U;
  My_ball_balance_debug_My.has_measurement = 0U;
  My_ball_speed_last_frame_count_My = 0U;
  My_ball_speed_reference_ms_My = 0U;
  My_ball_speed_reference_position_mm_My = 0;
  My_ball_speed_reference_ready_My = 0U;
  My_ball_balance_reset_control_My();
}

/**
  * @brief 让任务重新从 +50 mm 目标开始并清除上次控制状态。
  * @details 调用方负责与 TIM6 中断互斥；本函数不修改摄像头接收统计和最近测量值。
  */
void My_ball_balance_reset_control_My(void)
{
  My_ball_balance_debug_My.target_position_mm = MY_BALL_POSITIVE_TARGET_MM;
  My_ball_balance_debug_My.position_speed_mm_s = 0.0f;
  My_ball_balance_debug_My.measured_speed_mm_s = 0.0f;
  My_ball_balance_debug_My.distance_to_target_mm = 0.0f;
  My_ball_balance_debug_My.correction_angle_deg = 0.0f;
  My_ball_balance_debug_My.target_motor_angle_deg = MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG;
  My_ball_balance_debug_My.data_fresh = 0U;
  My_ball_balance_debug_My.position_speed_ready = 0U;
  My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
  My_ball_balance_debug_My.positive_target_reached = 0U;
  My_ball_balance_debug_My.final_arrival_start_ms = 0U;
  My_ball_balance_debug_My.final_arrival_settling = 0U;
  My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
  My_ball_balance_debug_My.negative_target_reached = 0U;
  My_ball_balance_debug_My.terminal_angle_commanded = 0U;
  My_ball_balance_debug_My.terminal_angle_reached = 0U;
  My_ball_speed_last_frame_count_My = My_ball_balance_debug_My.valid_frame_count;
  My_ball_speed_reference_ms_My = My_ball_balance_debug_My.last_update_ms;
  My_ball_speed_reference_position_mm_My = My_ball_balance_debug_My.position_mm;
  My_ball_speed_reference_ready_My = My_ball_balance_debug_My.has_measurement;
  My_ball_final_arrival_last_frame_count_My = My_ball_balance_debug_My.valid_frame_count;
}

/**
  * @brief 在主循环中解析 USART3 摄像头字节流并原子提交合法测量帧。
  * @param data 本次输入字节流首地址
  * @param length 本次输入字节数
  * @details 字符校验和重同步均在主循环完成；只在提交位置、速度和时间戳时短暂屏蔽中断。
  */
void My_ball_balance_feed_My(const uint8_t *data, uint16_t length)
{
  uint16_t data_index; /* 当前处理的 USART3 输入字节索引。 */

  if (data == NULL || length == 0U)
  {
    return;
  }

  for (data_index = 0U; data_index < length; data_index++)
  {
    uint8_t byte = data[data_index]; /* 本轮送入流式状态机的单字节。 */

    if (My_ball_parser_index_My == 0U)
    {
      if (byte == (uint8_t)'l')
      {
        My_ball_frame_buffer_My[0] = byte;
        My_ball_parser_index_My = 1U;
      }
    }
    else if (My_ball_parser_index_My == 1U)
    {
      if (byte == (uint8_t)'y')
      {
        My_ball_frame_buffer_My[1] = byte;
        My_ball_parser_index_My = 2U;
      }
      else if (byte != (uint8_t)'l')
      {
        My_ball_parser_index_My = 0U;
      }
    }
    else
    {
      My_ball_frame_buffer_My[My_ball_parser_index_My] = byte;
      My_ball_parser_index_My++;

      if (My_ball_parser_index_My == MY_BALL_FRAME_LENGTH)
      {
        int16_t position_mm; /* 当前候选帧解析得到的有符号位置。 */
        int16_t speed_mm_s;  /* 当前候选帧解析得到的有符号速度。 */

        if (My_ball_parse_frame_My(My_ball_frame_buffer_My,
                                   &position_mm,
                                   &speed_mm_s) != 0U)
        {
          uint32_t interrupt_mask = __get_PRIMASK(); /* 提交同一帧多字段状态前保存中断屏蔽状态。 */

          /*
           * TIM6 会同时读取位置、速度和时间戳。短暂屏蔽中断后整体提交，避免控制周期
           * 使用新位置搭配上一帧速度。临界区仅含自然对齐赋值，不执行解析或外设访问。
           */
          __disable_irq();
          My_ball_balance_debug_My.position_mm = position_mm;
          My_ball_balance_debug_My.speed_mm_s = speed_mm_s;
          My_ball_balance_debug_My.last_update_ms = HAL_GetTick();
          My_ball_balance_debug_My.valid_frame_count++;
          My_ball_balance_debug_My.has_measurement = 1U;
          if (interrupt_mask == 0U)
          {
            __enable_irq();
          }
          My_ball_parser_index_My = 0U;
        }
        else
        {
          My_ball_balance_debug_My.invalid_frame_count++;
          My_ball_resync_My();
        }
      }
    }

    My_ball_balance_debug_My.parser_index = My_ball_parser_index_My;
  }
}

/**
  * @brief 在 TIM6 的 10 ms 周期中执行 +50 mm 到 -50 mm 的三态控制。
  * @details 中断来源由上层回调确认。本函数读取主循环原子提交的测量快照，按距离和
  *          速度选择保持、固定小角度驱动或反向制动，再更新转向位置环目标。
  *          中断上下文中不执行阻塞、延时、打印、动态分配或不定长循环。
  */
void My_ball_balance_update_10ms_My(void)
{
  float requested_angle; /* 本周期按保持、驱动或制动动作直接生成的电机目标角。 */
  uint32_t now_ms = HAL_GetTick(); /* 本控制周期用于判断通信新鲜度的毫秒快照。 */

  /*
   * -40 mm 只负责触发终止角度目标；到达目标前继续使用平衡模式的最小驱动力
   * 克服机构静摩擦。实际角度进入允许误差后再切换普通定位模式，由其到位
   * 死区清零 PID 和 PWM。该判断位于通信新鲜度分支之前，丢帧后也能完成收敛。
   */
  if (My_ball_balance_debug_My.terminal_angle_commanded != 0U &&
      My_ball_balance_debug_My.terminal_angle_reached == 0U &&
      My_ball_abs_My(My_steering_get_actual_angle_My() -
        MY_BALL_TERMINAL_MOTOR_ANGLE_DEG) <=
        MY_BALL_TERMINAL_ANGLE_TOLERANCE_DEG)
  {
    My_ball_balance_debug_My.terminal_angle_reached = 1U;
    My_steering_set_balance_mode_My(0U);
  }

  /*
   * 原始位置、协议速度和时间戳由主循环在短临界区内整帧提交；当前运行在 TIM6
   * 中断中，不会被主循环抢占，因此可直接读取一致快照。控制速度优先由位置变化
   * 和实际帧间隔计算，避免协议速度长期为零时丢失制动依据；USART3 长时间没有
   * 合法帧时按通信超时处理，防止继续使用停留在端点的旧位置。
   */
  if (My_ball_balance_debug_My.has_measurement != 0U &&
      (uint32_t)(now_ms - My_ball_balance_debug_My.last_update_ms) <= MY_BALL_COMMUNICATION_TIMEOUT_MS)
  {
    float position_error; /* 当前阶段位置目标与摄像头测量值之差，单位毫米。 */
    float measured_speed; /* 位置差分优先、协议速度兜底的制动判断速度。 */
    float correction;     /* 三态规则选择出的固定驱动角、制动角或零修正。 */
    float drive_angle;    /* 当前目标阶段的驱动角，两段可分别现场整定。 */
    float brake_angle;    /* 当前目标阶段的制动角，第二段取软限位内可用的较大值。 */
    float max_speed;      /* 当前目标阶段允许的最高速度，第二段取更低值以提前控速。 */
    float brake_distance; /* 当前目标阶段的制动距离，第二段取更大值以补偿长行程惯性。 */
    uint8_t final_arrival_window; /* 非零表示 -50 mm 的位置和速度已进入最终到点窗口。 */

    measured_speed = My_ball_update_measured_speed_My();
    position_error = (float)My_ball_balance_debug_My.target_position_mm -
      (float)My_ball_balance_debug_My.position_mm; /* 当前阶段目标减当前位置。 */

    /*
     * 第一段只有在 +50 mm 位置误差和速度同时进入到点范围后才切换到 -50 mm。
     * 这样高速掠过 +50 mm 时仍保持当前目标并执行制动，不会把“经过”误判成“到达”。
     * 阶段标志只由本 TIM6 中断路径写入，不与主循环进行读改写共享。
     */
    if (My_ball_balance_debug_My.positive_target_reached == 0U &&
        My_ball_abs_My(position_error) <= MY_BALL_ARRIVE_DISTANCE_MM &&
        My_ball_abs_My(measured_speed) <= MY_BALL_ARRIVE_SPEED_MM_S)
    {
      My_ball_balance_debug_My.positive_target_reached = 1U;
      My_ball_balance_debug_My.target_position_mm = MY_BALL_NEGATIVE_TARGET_MM;
      position_error = (float)My_ball_balance_debug_My.target_position_mm -
        (float)My_ball_balance_debug_My.position_mm;
    }

    /*
     * 两段分别选择驱动角、制动角、速度阈值和制动距离。第二段行程约 100 mm，
     * 可独立降低驱动角或速度阈值并增大制动距离，避免调整第一段后相互影响。
     * 阶段参数不改变电机位置内环的角度软限位、PWM 限幅和方向保护。
     */
    if (My_ball_balance_debug_My.positive_target_reached == 0U)
    {
      drive_angle = MY_BALL_POSITIVE_DRIVE_ANGLE_DEG;
      brake_angle = MY_BALL_POSITIVE_BRAKE_ANGLE_DEG;
      max_speed = MY_BALL_POSITIVE_MAX_SPEED_MM_S;
      brake_distance = MY_BALL_POSITIVE_BRAKE_DISTANCE_MM;
    }
    else
    {
      drive_angle = MY_BALL_NEGATIVE_DRIVE_ANGLE_DEG;
      brake_angle = MY_BALL_NEGATIVE_BRAKE_ANGLE_DEG;
      max_speed = MY_BALL_NEGATIVE_MAX_SPEED_MM_S;
      brake_distance = MY_BALL_NEGATIVE_BRAKE_DISTANCE_MM;
    }

    final_arrival_window = (uint8_t)(
      My_ball_balance_debug_My.positive_target_reached != 0U &&
      My_ball_abs_My(position_error) <= MY_BALL_ARRIVE_DISTANCE_MM &&
      My_ball_abs_My(measured_speed) <= MY_BALL_ARRIVE_SPEED_MM_S);

    if (My_ball_balance_debug_My.terminal_angle_commanded != 0U)
    {
      /* 终止状态已锁存，持续下发终止角度，不再允许钢球外环修改电机目标。 */
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
      correction = MY_BALL_TERMINAL_MOTOR_ANGLE_DEG -
        MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG;
    }
    else if (My_ball_balance_debug_My.positive_target_reached != 0U &&
             My_ball_balance_debug_My.position_mm <= MY_BALL_TERMINAL_POSITION_MM &&
             My_ball_abs_My(measured_speed) <= MY_BALL_ARRIVE_SPEED_MM_S)
    {
      /* 到达 -40 mm 且速度已降到安全范围后，只触发一次终止角度指令。 */
      My_ball_balance_debug_My.terminal_angle_commanded = 1U;
      My_ball_balance_debug_My.terminal_angle_reached = 0U;
      My_ball_balance_debug_My.target_position_mm = MY_BALL_TERMINAL_POSITION_MM;
      My_ball_balance_debug_My.final_arrival_settling = 0U;
      My_ball_balance_debug_My.final_arrival_start_ms = 0U;
      My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
      position_error = (float)MY_BALL_TERMINAL_POSITION_MM -
        (float)My_ball_balance_debug_My.position_mm;
      correction = MY_BALL_TERMINAL_MOTOR_ANGLE_DEG -
        MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG;
    }
    else if (My_ball_balance_debug_My.negative_target_reached != 0U)
    {
      /* 最终到点已经锁存，后续摄像头位置量化噪声不得重新触发任何外环动作。 */
      My_ball_balance_debug_My.final_arrival_settling = 1U;
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
      correction = 0.0f;
    }
    else if (My_ball_balance_debug_My.positive_target_reached != 0U &&
             My_ball_balance_debug_My.final_arrival_settling == 0U &&
             final_arrival_window != 0U)
    {
      /* 第二段首次进入到点窗口时锁定平滑回水平流程，避免下一周期立刻重新制动。 */
      My_ball_balance_debug_My.final_arrival_settling = 1U;
      My_ball_balance_debug_My.final_arrival_start_ms = 0U;
      My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
      correction = My_ball_step_correction_to_level_My(
        My_ball_balance_debug_My.correction_angle_deg);
    }
    else if (My_ball_balance_debug_My.final_arrival_settling != 0U)
    {
      if (My_ball_abs_My(position_error) > MY_BALL_FINAL_HOLD_RELEASE_DISTANCE_MM)
      {
        /* 偏离最终目标超过滞回距离，说明确实离开终态窗口，恢复普通三态控制。 */
        My_ball_balance_debug_My.final_arrival_settling = 0U;
        My_ball_balance_debug_My.final_arrival_start_ms = 0U;
        My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
        correction = My_ball_select_correction_My(position_error,
                                                   measured_speed,
                                                   drive_angle,
                                                   brake_angle,
                                                   max_speed,
                                                   brake_distance);
      }
      else
      {
        /* 终态确认期间即使速度估计短时抖动，也只允许修正角单向回到水平。 */
        My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
        correction = My_ball_step_correction_to_level_My(
          My_ball_balance_debug_My.correction_angle_deg);
      }
    }
    else
    {
      correction = My_ball_select_correction_My(position_error,
                                                 measured_speed,
                                                 drive_angle,
                                                 brake_angle,
                                                 max_speed,
                                                 brake_distance);
    }

    if (My_ball_balance_debug_My.positive_target_reached != 0U &&
        My_ball_balance_debug_My.terminal_angle_commanded == 0U &&
        My_ball_balance_debug_My.negative_target_reached == 0U &&
        My_ball_balance_debug_My.final_arrival_settling != 0U)
    {
      /* 只用新合法帧累计确认次数，禁止同一帧在多个 10 ms 中断中重复计数。 */
      if (My_ball_balance_debug_My.valid_frame_count !=
          My_ball_final_arrival_last_frame_count_My)
      {
        My_ball_final_arrival_last_frame_count_My =
          My_ball_balance_debug_My.valid_frame_count;
        if (correction == 0.0f &&
            My_ball_abs_My(position_error) <= MY_BALL_ARRIVE_DISTANCE_MM &&
            My_ball_abs_My(measured_speed) <= MY_BALL_ARRIVE_SPEED_MM_S)
        {
          if (My_ball_balance_debug_My.final_arrival_confirm_count == 0U)
          {
            My_ball_balance_debug_My.final_arrival_start_ms =
              My_ball_balance_debug_My.last_update_ms;
          }
          if (My_ball_balance_debug_My.final_arrival_confirm_count <
              MY_BALL_FINAL_ARRIVE_CONFIRM_FRAMES)
          {
            My_ball_balance_debug_My.final_arrival_confirm_count++;
          }
          if (My_ball_balance_debug_My.final_arrival_confirm_count >=
                MY_BALL_FINAL_ARRIVE_CONFIRM_FRAMES &&
              (uint32_t)(My_ball_balance_debug_My.last_update_ms -
                My_ball_balance_debug_My.final_arrival_start_ms) >=
                MY_BALL_FINAL_ARRIVE_CONFIRM_MS)
          {
            /* 最终完成只允许单向锁存，后续单帧量化噪声不能重新触发驱动或制动。 */
            My_ball_balance_debug_My.negative_target_reached = 1U;
            My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
          }
        }
        else
        {
          /* 新帧不满足完整到点条件时，连续确认必须从零重新开始。 */
          My_ball_balance_debug_My.final_arrival_start_ms = 0U;
          My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
        }
      }
    }
    My_ball_balance_debug_My.data_fresh = 1U;
    My_ball_balance_debug_My.measured_speed_mm_s = measured_speed;
    My_ball_balance_debug_My.distance_to_target_mm = My_ball_abs_My(position_error);
    My_ball_balance_debug_My.correction_angle_deg = correction;
    requested_angle = MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG + correction;
  }
  else
  {
    /*
     * 无首帧或通信超时时不继续使用旧状态；清除测速参考并把平台目标直接设为水平。
     * 下层转向位置环仍负责角度软限位和 PWM 限幅，因此这里不访问外设、不阻塞。
     */
    My_ball_balance_debug_My.data_fresh = 0U;
    My_ball_balance_debug_My.position_speed_mm_s = 0.0f;
    My_ball_balance_debug_My.measured_speed_mm_s = 0.0f;
    My_ball_balance_debug_My.position_speed_ready = 0U;
    My_ball_balance_debug_My.distance_to_target_mm = 0.0f;
    My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
    if (My_ball_balance_debug_My.negative_target_reached == 0U)
    {
      /* 未最终到点时通信中断会使连续确认作废，恢复通信后必须从新帧重新确认。 */
      My_ball_balance_debug_My.final_arrival_settling = 0U;
      My_ball_balance_debug_My.final_arrival_start_ms = 0U;
      My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
      My_ball_final_arrival_last_frame_count_My =
        My_ball_balance_debug_My.valid_frame_count;
    }
    My_ball_speed_reference_ready_My = 0U;
    if (My_ball_balance_debug_My.terminal_angle_commanded != 0U)
    {
      /* 终止角度已经由有效数据触发，即使后续丢帧也保持该目标完成收敛。 */
      My_ball_balance_debug_My.correction_angle_deg =
        MY_BALL_TERMINAL_MOTOR_ANGLE_DEG - MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG;
      requested_angle = MY_BALL_TERMINAL_MOTOR_ANGLE_DEG;
    }
    else
    {
      My_ball_balance_debug_My.correction_angle_deg = 0.0f;
      requested_angle = MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG;
    }
  }

  /*
   * 制动条件成立后直接切换角度目标，不再叠加外环变化率延迟；下层位置环继续执行
   * 角度软限位、PID 输出限幅和方向切换前 PWM 清零，避免绕过既有机构保护。
   */
  My_ball_balance_debug_My.target_motor_angle_deg = requested_angle;
  My_steering_set_target_angle_My(requested_angle);
}
