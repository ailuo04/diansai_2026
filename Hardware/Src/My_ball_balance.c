#include "My_ball_balance.h"

#include "My_steering.h"
#include "stm32f4xx_hal.h"

#include <string.h>

#define MY_BALL_POSITION_ABS_LIMIT_MM 125 /* 摄像头协议定义的平台中心到左右端点最大距离。 */

volatile My_ball_balance_debug_t My_ball_balance_debug_My; /* 供 Keil Watch 观察的摄像头和外环状态。 */
volatile My_pid_t My_ball_task3_pid_My; /* 任务 3 分段目标追踪专用的位置式 PID 参数和历史状态。 */
volatile My_pid_t My_ball_zero_pid_My; /* 任务 5/6 共用的钢球位置 PID 参数和历史状态。 */
static volatile My_pid_t My_ball_task4_pid_My; /* 任务 4 独立的钢球 0 点 PID 参数和历史状态。 */

static uint8_t My_ball_frame_buffer_My[MY_BALL_FRAME_LENGTH]; /* 主循环流式重组中的候选固定长度帧。 */
static uint8_t My_ball_parser_index_My; /* 候选帧已缓存字节数，仅由主循环访问。 */
static uint32_t My_ball_final_arrival_last_frame_count_My; /* 最终到点确认已经检查过的合法帧序号。 */
static float My_ball_task3_correction_My; /* 任务 3 PID 上一周期实际下发的电机输出轴修正角。 */
static float My_ball_zero_correction_My; /* 任务 5/6 共用 PID 上一周期实际下发的电机输出轴修正角。 */
static float My_ball_task4_correction_My; /* 任务 4 独立 PID 上一周期实际下发的电机输出轴修正角。 */
static float My_ball_vehicle_accel_target_My; /* 任务 4/5/6 底盘速度规划给出的纵向加速度前馈输入。 */
static int32_t My_ball_start_target_sum_My; /* 任务 6 启动采样位置累计和。 */
static uint8_t My_ball_start_target_count_My; /* 任务 6 已累计的合法帧数。 */
static volatile uint8_t My_ball_start_target_sampling_My; /* 非零表示主循环应把新合法帧计入任务 6 目标。 */
static volatile uint8_t My_ball_start_target_ready_My; /* 非零表示 TIM6 可使用任务 6 锁存目标。 */

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
  * @brief 按固定最大步长把当前值移动到目标值。
  * @param current_value 当前值
  * @param target_value 目标值
  * @param maximum_step 单周期允许变化的正步长
  * @retval 完成单周期变化率限制后的新值
  * @details 本函数由 TIM6 中断路径调用，只进行定长比较和加减运算；用于阻止
  *          钢球位置 PID 目标角因测量噪声、目标切换或通信恢复产生瞬时大阶跃。
  */
static float My_ball_move_toward_My(float current_value,
                                    float target_value,
                                    float maximum_step)
{
  if (target_value > current_value + maximum_step)
  {
    return current_value + maximum_step;
  }
  if (target_value < current_value - maximum_step)
  {
    return current_value - maximum_step;
  }

  return target_value;
}

/**
  * @brief 按指定参数重新装载钢球位置外环 PID 并清除输出历史。
  * @param pid 当前任务独占的 PID 参数和历史状态
  * @param correction 当前任务独占的上一周期实际修正角
  * @param kp 位置误差比例增益
  * @param ki 位置误差时间积分增益
  * @param kd 摄像头速度微分反馈增益
  * @param integral_limit 位置误差积分限幅
  * @param output_limit 输出轴修正角限幅
  * @details 任务切换前由主循环在外部临界区调用，也会在初始化阶段调用。本函数只
  *          更新 PID 参数和私有修正角，不访问外设、不阻塞；避免沿用上一次运行的
  *          积分、微分历史和角度输出。
  */
static void My_ball_load_pid_state_My(volatile My_pid_t *pid,
                                      float *correction,
                                      float kp,
                                      float ki,
                                      float kd,
                                      float integral_limit,
                                      float output_limit)
{
  My_pid_init_My(pid,
                 kp,
                 ki,
                 kd,
                 integral_limit,
                 output_limit);
  *correction = 0.0f;
}

/**
  * @brief 装载任务 3 分段钢球控制专用 PID 参数。
  * @details 任务 3 目标会从 +50 mm 切到 -50 mm，参数独立于任务 4/5 的 0 点保持；
  *          进入任务 3 时调用本函数可避免 0 点保持调参影响分段追踪。
  */
static void My_ball_load_task3_pid_state_My(void)
{
  My_ball_load_pid_state_My(&My_ball_task3_pid_My,
                            &My_ball_task3_correction_My,
                            MY_BALL_TASK3_PID_KP_DEG_PER_MM,
                            MY_BALL_TASK3_PID_KI_DEG_PER_MM_S,
                            MY_BALL_TASK3_PID_KD_DEG_PER_MM_S,
                            MY_BALL_TASK3_INTEGRAL_LIMIT_MM_S,
                            MY_BALL_TASK3_CORRECTION_LIMIT_DEG);
}

/**
  * @brief 装载任务 5/6 共用的钢球位置 PID 参数。
  * @details 任务 5 固定保持 0 mm，任务 6 保持启动采样目标；两项任务使用同一组
  *          PID 参数，但每次进入任务时都会清除历史，避免沿用上一次任务的积分量。
  */
static void My_ball_load_task56_pid_state_My(void)
{
  My_ball_load_pid_state_My(&My_ball_zero_pid_My,
                            &My_ball_zero_correction_My,
                            MY_BALL_TASK56_PID_KP_DEG_PER_MM,
                            MY_BALL_TASK56_PID_KI_DEG_PER_MM_S,
                            MY_BALL_TASK56_PID_KD_DEG_PER_MM_S,
                            MY_BALL_TASK56_INTEGRAL_LIMIT_MM_S,
                            MY_BALL_TASK56_CORRECTION_LIMIT_DEG);
}

/**
  * @brief 装载任务 4 独立的钢球 0 点保持 PID 参数。
  * @details 任务 4 的参数和历史均不与任务 5/6 共用，便于单独适配定时 S 曲线。
  */
static void My_ball_load_task4_pid_state_My(void)
{
  My_ball_load_pid_state_My(&My_ball_task4_pid_My,
                            &My_ball_task4_correction_My,
                            MY_BALL_TASK4_PID_KP_DEG_PER_MM,
                            MY_BALL_TASK4_PID_KI_DEG_PER_MM_S,
                            MY_BALL_TASK4_PID_KD_DEG_PER_MM_S,
                            MY_BALL_TASK4_INTEGRAL_LIMIT_MM_S,
                            MY_BALL_TASK4_CORRECTION_LIMIT_DEG);
}

/**
  * @brief 使用摄像头位置和速度计算当前任务已装载参数的位置式 PID 修正角。
  * @param pid 当前任务独占的 PID 参数和历史状态
  * @param correction 当前任务独占的上一周期实际修正角
  * @param target_position_mm 当前目标位置，单位毫米
  * @param measured_position_mm 摄像头返回的位置，单位毫米
  * @param measured_speed_mm_s 摄像头同一帧返回的速度，单位毫米每秒
  * @param integral_distance_mm 允许积分的位置误差绝对值上限
  * @param integral_speed_mm_s 允许积分的速度绝对值上限
  * @param integral_decay 暂停新增积分时的历史积分释放比例
  * @param correction_step_deg 单周期修正角变化上限
  * @retval 完成输出限幅和单周期变化率限制后的电机输出轴修正角
  * @details 任务切换时已提前装载任务 3 或任务 4/5 的独立 PID 参数。P/I 项使用位置
  *          误差，D 项直接使用摄像头速度；只在目标附近且低速时积分，其他状态逐步
  *          释放积分。函数只执行定长浮点运算，由 TIM6 中断调用，不阻塞、不打印、
  *          不分配内存，也不访问通信外设。
  */
static float My_ball_calculate_pid_correction_My(volatile My_pid_t *pid,
                                                 float *correction,
                                                 float target_position_mm,
                                                 float measured_position_mm,
                                                 float measured_speed_mm_s,
                                                 float integral_distance_mm,
                                                 float integral_speed_mm_s,
                                                 float integral_decay,
                                                 float correction_step_deg)
{
  float position_error = target_position_mm - measured_position_mm; /* 当前目标位置误差。 */
  float integration_period; /* 满足积分条件时为 10 ms，否则为零以暂停新增积分。 */
  float pid_output; /* 完成抗饱和和限幅后、尚未映射机构方向的 PID 输出。 */
  float requested_correction; /* 完成机构方向映射、尚未进行变化率限制的修正角。 */

  if (My_ball_abs_My(position_error) <= integral_distance_mm &&
      My_ball_abs_My(measured_speed_mm_s) <= integral_speed_mm_s)
  {
    integration_period = MY_BALL_ZERO_CONTROL_PERIOD_S;
  }
  else
  {
    /* 大偏差或高速运动时释放历史积分，防止换向和制动阶段出现积分拖尾。 */
    pid->integral *= integral_decay;
    integration_period = 0.0f;
  }

  pid_output = My_pid_calc_position_with_derivative_My(
    pid,
    target_position_mm,
    measured_position_mm,
    measured_speed_mm_s,
    integration_period);
  requested_correction = MY_BALL_CONTROL_DIRECTION * pid_output;

  /* 分项调试值与实际控制使用同一组 PID 状态，便于通过 Keil Watch 现场整定。 */
  My_ball_balance_debug_My.pid_p_angle_deg = MY_BALL_CONTROL_DIRECTION *
    pid->kp * pid->error;
  My_ball_balance_debug_My.pid_i_angle_deg = MY_BALL_CONTROL_DIRECTION *
    pid->ki * pid->integral;
  My_ball_balance_debug_My.pid_d_angle_deg = MY_BALL_CONTROL_DIRECTION *
    (-pid->kd * measured_speed_mm_s);
  My_ball_balance_debug_My.pid_output_angle_deg = requested_correction;

  *correction = My_ball_move_toward_My(*correction,
                                      requested_correction,
                                      correction_step_deg);
  return *correction;
}

/**
  * @brief 按任务 4/5 的规划加速度计算平台倾角前馈。
  * @param accel_target_per_s 速度目标每秒变化量
  * @retval 限幅后的电机输出轴前馈角，单位度
  * @details 当前工程没有底盘真实水平加速度传感器，因此先使用可调比例把规划
  *          加速度映射为小角度前馈。该前馈不能替代后续 IMU 实测加速度补偿。
  */
static float My_ball_calculate_accel_feedforward_My(float accel_target_per_s,
                                                    float angle_limit_deg)
{
  return My_ball_limit_symmetric_My(
    MY_BALL_ACCEL_FF_DIRECTION *
    MY_BALL_ACCEL_FF_DEG_PER_TARGET_ACCEL *
    accel_target_per_s,
    angle_limit_deg);
}

/**
  * @brief 初始化摄像头帧解析、位置速度反馈和钢球 PID 控制状态。
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
  My_ball_balance_reset_control_My();
}

/**
  * @brief 让任务重新从 +50 mm 目标开始并清除上次控制状态。
  * @details 调用方负责与 TIM6 中断互斥；本函数不修改摄像头接收统计和最近测量值。
  */
void My_ball_balance_reset_control_My(void)
{
  /* 任何任务重新进入通用复位时先停止任务 6 采样，防止后续帧误改其他任务目标。 */
  My_ball_start_target_sampling_My = 0U;
  My_ball_start_target_ready_My = 0U;
  My_ball_balance_debug_My.target_position_mm = MY_BALL_POSITIVE_TARGET_MM;
  My_ball_balance_debug_My.position_speed_mm_s = 0.0f;
  My_ball_balance_debug_My.measured_speed_mm_s = 0.0f;
  My_ball_balance_debug_My.pid_p_angle_deg = 0.0f;
  My_ball_balance_debug_My.pid_i_angle_deg = 0.0f;
  My_ball_balance_debug_My.pid_d_angle_deg = 0.0f;
  My_ball_balance_debug_My.pid_output_angle_deg = 0.0f;
  My_ball_balance_debug_My.vehicle_accel_target = 0.0f;
  My_ball_balance_debug_My.accel_feedforward_angle_deg = 0.0f;
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
  My_ball_vehicle_accel_target_My = 0.0f;
  My_ball_balance_debug_My.start_target_ready = 0U;
  My_ball_balance_debug_My.start_target_sample_count = 0U;
  My_ball_final_arrival_last_frame_count_My = My_ball_balance_debug_My.valid_frame_count;
  My_ball_load_task3_pid_state_My();
}

/**
  * @brief 复位任务 5 的钢球 0 点保持状态，并装载任务 5/6 共用参数。
  * @details 先复用任务 3 的完整状态清理，再把目标改为 0 mm；本函数不访问阻塞
  *          外设，调用方负责与 TIM6 中断互斥，避免复位过程与周期控制并发。
  */
void My_ball_balance_reset_zero_control_My(void)
{
  My_ball_balance_reset_control_My();
  My_ball_balance_debug_My.target_position_mm = MY_BALL_ZERO_TARGET_MM;
  My_ball_vehicle_accel_target_My = 0.0f;
  My_ball_load_task56_pid_state_My();
}

/**
  * @brief 复位任务 4 独立的钢球 0 点控制状态。
  * @details 调用方已在任务确认临界区内屏蔽 TIM6。本函数先清理通用任务状态，再
  *          装载任务 4 专用 PID 参数和修正角历史；不访问外设、不阻塞、不分配内存。
  */
void My_ball_balance_reset_task4_control_My(void)
{
  My_ball_balance_reset_control_My();
  My_ball_balance_debug_My.target_position_mm = MY_BALL_ZERO_TARGET_MM;
  My_ball_vehicle_accel_target_My = 0.0f;
  My_ball_load_task4_pid_state_My();
}

void My_ball_balance_set_vehicle_accel_feedforward_My(float accel_target_per_s)
{
  /*
   * 规划加速度由任务 4 或任务 5 的 10 ms 控制路径写入，并在同一个 TIM6 中断序列内
   * 被钢球外环读取。任务切换和停车流程会显式写入 0，避免前馈角遗留到静止保持阶段。
   */
  My_ball_vehicle_accel_target_My = accel_target_per_s;
}

/**
  * @brief 复位任务 6 启动帧目标采样和平衡控制状态。
  * @details 调用方已在任务切换临界区内屏蔽 TIM6。函数清除旧 PID、累计和及锁存
  *          状态，并从调用后的下一帧合法摄像头数据开始采样；不访问外设、不阻塞。
  */
void My_ball_balance_reset_start_target_control_My(void)
{
  My_ball_balance_reset_control_My();
  My_ball_balance_debug_My.target_position_mm =
    (My_ball_balance_debug_My.has_measurement != 0U) ?
      My_ball_balance_debug_My.position_mm : MY_BALL_ZERO_TARGET_MM;
  My_ball_start_target_sum_My = 0;
  My_ball_start_target_count_My = 0U;
  My_ball_start_target_ready_My = 0U;
  My_ball_start_target_sampling_My = 1U;
  My_ball_balance_debug_My.start_target_ready = 0U;
  My_ball_balance_debug_My.start_target_sample_count = 0U;
  My_ball_load_task56_pid_state_My();
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

          /*
           * 任务 6 的“最开始几帧”必须在每个合法帧提交时累计，不能等待 TIM6
           * 轮询，否则一次 DMA 数据中包含多帧时会漏掉中间帧。累计和最大为
           * 5×125 mm，远低于 int32_t 上限；达到帧数后用对称四舍五入锁存目标，
           * 随即关闭采样，后续帧只能更新测量值，不能让目标随钢球漂移。
           */
          if (My_ball_start_target_sampling_My != 0U)
          {
            My_ball_start_target_sum_My += (int32_t)position_mm;
            My_ball_start_target_count_My++;
            My_ball_balance_debug_My.start_target_sample_count =
              My_ball_start_target_count_My;
            if (My_ball_start_target_count_My >=
                MY_BALL_START_TARGET_SAMPLE_FRAMES)
            {
              int32_t rounding =
                (int32_t)(MY_BALL_START_TARGET_SAMPLE_FRAMES / 2U);
              int32_t target_mm; /* 对有符号位置和完成对称四舍五入后的目标点。 */

              target_mm = (My_ball_start_target_sum_My >= 0) ?
                (My_ball_start_target_sum_My + rounding) /
                  (int32_t)MY_BALL_START_TARGET_SAMPLE_FRAMES :
                (My_ball_start_target_sum_My - rounding) /
                  (int32_t)MY_BALL_START_TARGET_SAMPLE_FRAMES;
              My_ball_balance_debug_My.target_position_mm =
                (int16_t)target_mm;
              My_ball_start_target_sampling_My = 0U;
              My_ball_start_target_ready_My = 1U;
              My_ball_balance_debug_My.start_target_ready = 1U;
              My_pid_reset_My(&My_ball_zero_pid_My);
              My_ball_zero_correction_My = 0.0f;
            }
          }
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
  * @brief 在 TIM6 的 10 ms 周期中执行任务 3 的 +50 mm 到 -50 mm 分段位置式 PID。
  * @details 中断来源由上层回调确认。本函数读取主循环原子提交的摄像头位置、速度和
  *          时间戳，两段均调用任务 4/5 同款 PID；到达 +50 mm 后清除 PID 历史并切换
  *          -50 mm 目标，到达 -50 mm 后继续以该位置实时闭环，补偿钢球扰动和静差。
  *          中断中不阻塞、不打印、不分配动态内存，也不访问慢速通信外设。
  */
void My_ball_balance_update_10ms_My(void)
{
  float requested_angle = MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG; /* 本周期下发给转向位置环的绝对目标角。 */
  uint32_t now_ms = HAL_GetTick(); /* 本周期用于判断摄像头帧新鲜度的毫秒快照。 */

  /*
   * 位置、速度和时间戳由主循环在短临界区内按同一摄像头帧整体提交。当前运行在
   * TIM6 中断中，主循环不能并发改写，因此可直接读取一致快照；超过通信时限后
   * 禁止继续使用旧位置和旧速度。
   */
  if (My_ball_balance_debug_My.has_measurement != 0U &&
      (uint32_t)(now_ms - My_ball_balance_debug_My.last_update_ms) <=
        MY_BALL_COMMUNICATION_TIMEOUT_MS)
  {
    float measured_speed = My_ball_limit_symmetric_My(
      (float)My_ball_balance_debug_My.speed_mm_s,
      MY_BALL_SPEED_ABS_LIMIT_MM_S); /* 摄像头帧速度经物理限幅后的 PID 速度反馈。 */
    float position_error = (float)My_ball_balance_debug_My.target_position_mm -
      (float)My_ball_balance_debug_My.position_mm; /* 当前分段目标减摄像头位置。 */
    float correction; /* 任务 3 专用 PID 计算并完成变化率限制后的平台修正角。 */
    uint8_t final_arrival_window; /* 非零表示 -50 mm 的位置和速度均进入到点窗口。 */

    /*
     * 第一段位置进入 40～50 mm 区间就立即切换目标，不再等待 +50 mm 完全稳定；
     * 目标阶跃不会产生微分冲击，因为 D 项直接使用实际速度；仍需清除第一段积分，
     * 防止其方向与第二段初始控制相反。
     */
    if (My_ball_balance_debug_My.positive_target_reached == 0U &&
        My_ball_balance_debug_My.position_mm >= MY_BALL_POSITIVE_SWITCH_MIN_MM &&
        My_ball_balance_debug_My.position_mm <= MY_BALL_POSITIVE_TARGET_MM)
    {
      My_ball_balance_debug_My.positive_target_reached = 1U;
      My_ball_balance_debug_My.target_position_mm = MY_BALL_NEGATIVE_TARGET_MM;
      My_pid_reset_My(&My_ball_task3_pid_My);
      position_error = (float)MY_BALL_NEGATIVE_TARGET_MM -
        (float)My_ball_balance_debug_My.position_mm;
    }

    final_arrival_window = (uint8_t)(
      My_ball_balance_debug_My.positive_target_reached != 0U &&
      My_ball_abs_My(position_error) <= MY_BALL_ARRIVE_DISTANCE_MM &&
      My_ball_abs_My(measured_speed) <= MY_BALL_ARRIVE_SPEED_MM_S);

    correction = My_ball_calculate_pid_correction_My(
      &My_ball_task3_pid_My,
      &My_ball_task3_correction_My,
      (float)My_ball_balance_debug_My.target_position_mm,
      (float)My_ball_balance_debug_My.position_mm,
      measured_speed,
      MY_BALL_TASK3_INTEGRAL_DISTANCE_MM,
      MY_BALL_TASK3_INTEGRAL_SPEED_MM_S,
      MY_BALL_TASK3_INTEGRAL_DECAY,
      MY_BALL_TASK3_CORRECTION_STEP_DEG);

    if (final_arrival_window != 0U)
    {
      /* 到点只改变调试状态，PID 仍持续输出以补偿 -50 mm 附近的扰动和静差。 */
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
    }
    else if (measured_speed *
               (MY_BALL_CONTROL_DIRECTION * correction) < 0.0f)
    {
      /* PID 作用方向与钢球运动方向相反时，当前周期正在主动制动。 */
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_BRAKE;
    }
    else
    {
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_DRIVE;
    }

    /*
     * 第二段只按新合法帧累计连续到点次数，禁止同一帧被多个 10 ms 中断重复计数。
     * 达到完整位置、速度和持续时间条件后只锁存到达状态，不退出 -50 mm 外环。
     */
    if (My_ball_balance_debug_My.positive_target_reached != 0U &&
        My_ball_balance_debug_My.valid_frame_count !=
          My_ball_final_arrival_last_frame_count_My)
    {
      My_ball_final_arrival_last_frame_count_My =
        My_ball_balance_debug_My.valid_frame_count;
      My_ball_balance_debug_My.final_arrival_settling = final_arrival_window;
      if (final_arrival_window != 0U)
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
          /* 锁存“已到达”仅供任务状态和现场观察使用，控制输出继续由 PID 实时更新。 */
          My_ball_balance_debug_My.negative_target_reached = 1U;
        }
      }
      else
      {
        /* 任一新帧离开到点窗口，连续确认从零开始但 PID 继续追踪 -50 mm。 */
        My_ball_balance_debug_My.final_arrival_start_ms = 0U;
        My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
      }
    }

    My_ball_balance_debug_My.data_fresh = 1U;
    My_ball_balance_debug_My.position_speed_mm_s = measured_speed;
    My_ball_balance_debug_My.measured_speed_mm_s = measured_speed;
    My_ball_balance_debug_My.position_speed_ready = 1U;
    My_ball_balance_debug_My.distance_to_target_mm = My_ball_abs_My(position_error);
    My_ball_balance_debug_My.correction_angle_deg = correction;
    requested_angle = MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG + correction;
  }
  else
  {
    /*
     * 尚未收到首帧或通信超时时，清除 PID、速度和连续到点确认，并按正常变化率平滑
     * 回到水平。恢复有效数据后仍以当前分段目标重新进入闭环。
     */
    My_ball_balance_debug_My.data_fresh = 0U;
    My_ball_balance_debug_My.position_speed_mm_s = 0.0f;
    My_ball_balance_debug_My.measured_speed_mm_s = 0.0f;
    My_ball_balance_debug_My.position_speed_ready = 0U;
    My_ball_balance_debug_My.distance_to_target_mm = 0.0f;
    My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
    My_pid_reset_My(&My_ball_task3_pid_My);
    My_ball_balance_debug_My.pid_p_angle_deg = 0.0f;
    My_ball_balance_debug_My.pid_i_angle_deg = 0.0f;
    My_ball_balance_debug_My.pid_d_angle_deg = 0.0f;
    My_ball_balance_debug_My.pid_output_angle_deg = 0.0f;
    if (My_ball_balance_debug_My.negative_target_reached == 0U)
    {
      My_ball_balance_debug_My.final_arrival_settling = 0U;
      My_ball_balance_debug_My.final_arrival_start_ms = 0U;
      My_ball_balance_debug_My.final_arrival_confirm_count = 0U;
      My_ball_final_arrival_last_frame_count_My =
        My_ball_balance_debug_My.valid_frame_count;
    }

    My_ball_task3_correction_My = My_ball_move_toward_My(
      My_ball_task3_correction_My,
      0.0f,
      MY_BALL_TASK3_CORRECTION_STEP_DEG);
    My_ball_balance_debug_My.correction_angle_deg = My_ball_task3_correction_My;
    requested_angle += My_ball_task3_correction_My;
  }

  /* 下层位置环继续执行角度软限位、方向切换保护和 PWM 限幅。 */
  My_ball_balance_debug_My.target_motor_angle_deg = requested_angle;
  My_steering_set_target_angle_My(requested_angle);
}

/**
  * @brief 在 TIM6 的 10 ms 周期中执行任务 4/5/6 钢球目标位置式 PID。
  * @param target_position_mm 本周期使用的固定钢球目标位置
  * @details 本函数在任务行驶和停车阶段持续执行。它读取主循环原子提交的
  *          摄像头位置、速度和时间戳，将位置误差送入 P/I 项、返回速度送入 D 项，
  *          并限制电机目标角幅值和变化率。中断内不阻塞、不打印、
  *          不分配动态内存，也不直接读写慢速通信外设。
  */
static void My_ball_balance_update_target_10ms_My(int16_t target_position_mm,
                                                  volatile My_pid_t *pid,
                                                  float *correction,
                                                  float accel_ff_limit_deg)
{
  float requested_angle = MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG; /* 本周期下发给转向位置环的绝对目标角度。 */
  float accel_feedforward_angle = My_ball_calculate_accel_feedforward_My(
    My_ball_vehicle_accel_target_My,
    accel_ff_limit_deg); /* 由当前任务规划加速度和专用绝对限幅得到的惯性补偿前馈角。 */
  uint32_t now_ms = HAL_GetTick(); /* 用于判断摄像头数据是否超过 200 ms 的统一时间快照。 */

  /*
   * 位置、速度和到达时刻由主循环在短临界区内按同一帧整体提交。当前处于 TIM6
   * 中断上下文，主循环不会并发改写；只接受未超时的完整测量。协议速度先做物理
   * 上限约束再直接送入 D 项，不进行位置差分或观测器替代。
   */
  if (My_ball_balance_debug_My.has_measurement != 0U &&
      (uint32_t)(now_ms - My_ball_balance_debug_My.last_update_ms) <=
        MY_BALL_COMMUNICATION_TIMEOUT_MS)
  {
    float position_error; /* 固定目标减观测位置，单位毫米。 */
    float measured_speed; /* 摄像头同一帧返回并完成物理限幅的钢球速度。 */

    measured_speed = My_ball_limit_symmetric_My(
      (float)My_ball_balance_debug_My.speed_mm_s,
      MY_BALL_SPEED_ABS_LIMIT_MM_S);
    position_error = (float)target_position_mm -
      (float)My_ball_balance_debug_My.position_mm;
    *correction = My_ball_calculate_pid_correction_My(
      pid,
      correction,
      (float)target_position_mm,
      (float)My_ball_balance_debug_My.position_mm,
      measured_speed,
      MY_BALL_ZERO_INTEGRAL_DISTANCE_MM,
      MY_BALL_ZERO_INTEGRAL_SPEED_MM_S,
      MY_BALL_ZERO_INTEGRAL_DECAY,
      MY_BALL_ZERO_CORRECTION_STEP_DEG);

    if (My_ball_abs_My(position_error) <= MY_BALL_ZERO_ARRIVE_DISTANCE_MM &&
        My_ball_abs_My(measured_speed) <=
          MY_BALL_ZERO_ARRIVE_SPEED_MM_S)
    {
      /* 位置和速度均进入中心窗口时标记保持，但连续控制仍可补偿小静差。 */
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
    }
    else if (measured_speed *
               (MY_BALL_CONTROL_DIRECTION * *correction) < 0.0f)
    {
      /* 控制作用与当前运动方向相反时标记制动，便于现场观察阻尼是否及时介入。 */
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_BRAKE;
    }
    else
    {
      My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_DRIVE;
    }

    My_ball_balance_debug_My.data_fresh = 1U;
    My_ball_balance_debug_My.target_position_mm = target_position_mm;
    My_ball_balance_debug_My.position_speed_mm_s = measured_speed;
    My_ball_balance_debug_My.measured_speed_mm_s = measured_speed;
    My_ball_balance_debug_My.position_speed_ready = 1U;
    My_ball_balance_debug_My.distance_to_target_mm = My_ball_abs_My(position_error);
    My_ball_balance_debug_My.correction_angle_deg = *correction;
    My_ball_balance_debug_My.vehicle_accel_target = My_ball_vehicle_accel_target_My;
    My_ball_balance_debug_My.accel_feedforward_angle_deg = accel_feedforward_angle;
    requested_angle = MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG +
      *correction + accel_feedforward_angle;
  }
  else
  {
    /*
     * 尚未收到首帧或通信超时时，禁止继续沿用旧观测状态；清除速度和积分，并按
     * 与正常控制相同的变化率逐步回到水平，避免丢帧瞬间产生反向角度阶跃。该分支
     * 只写控制状态，角度软限位和 PWM 限幅仍由位置内环负责。
     */
    My_ball_balance_debug_My.data_fresh = 0U;
    My_ball_balance_debug_My.target_position_mm = target_position_mm;
    My_ball_balance_debug_My.position_speed_mm_s = 0.0f;
    My_ball_balance_debug_My.measured_speed_mm_s = 0.0f;
    My_ball_balance_debug_My.position_speed_ready = 0U;
    My_ball_balance_debug_My.distance_to_target_mm = 0.0f;
    My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
    My_pid_reset_My(pid);
    My_ball_balance_debug_My.pid_p_angle_deg = 0.0f;
    My_ball_balance_debug_My.pid_i_angle_deg = 0.0f;
    My_ball_balance_debug_My.pid_d_angle_deg = 0.0f;
    My_ball_balance_debug_My.pid_output_angle_deg = 0.0f;
    *correction = My_ball_move_toward_My(
      *correction,
      0.0f,
      MY_BALL_ZERO_CORRECTION_STEP_DEG);
    My_ball_balance_debug_My.correction_angle_deg = *correction;
    My_ball_balance_debug_My.vehicle_accel_target = My_ball_vehicle_accel_target_My;
    My_ball_balance_debug_My.accel_feedforward_angle_deg = accel_feedforward_angle;
    requested_angle += *correction + accel_feedforward_angle;
  }

  /* 下层位置环继续执行角度软限位、方向切换保护和 PWM 限幅。 */
  My_ball_balance_debug_My.target_motor_angle_deg = requested_angle;
  My_steering_set_target_angle_My(requested_angle);
}

/**
  * @brief 在 TIM6 的 10 ms 周期中执行任务 5 钢球零点位置式 PID。
  * @details 任务 5 行驶和停车后均固定保持相机坐标 0 mm，并与任务 6 共用 PID
  *          参数和加速度前馈角绝对限幅；本函数不访问慢速外设、不阻塞、不打印。
  */
void My_ball_balance_update_zero_10ms_My(void)
{
  My_ball_balance_update_target_10ms_My(MY_BALL_ZERO_TARGET_MM,
                                        &My_ball_zero_pid_My,
                                        &My_ball_zero_correction_My,
                                        MY_BALL_TASK56_ACCEL_FF_LIMIT_DEG);
}

/**
  * @brief 在 TIM6 的 10 ms 周期中执行任务 4 独立的钢球零点位置式 PID。
  * @details 任务 4 行驶和停车后均持续保持相机坐标 0 mm，控制过程只使用任务 4
  *          专用 PID 状态与前馈角绝对限幅；中断内不阻塞、不打印、不分配内存。
  */
void My_ball_balance_update_task4_10ms_My(void)
{
  My_ball_balance_update_target_10ms_My(MY_BALL_ZERO_TARGET_MM,
                                        &My_ball_task4_pid_My,
                                        &My_ball_task4_correction_My,
                                        MY_BALL_TASK4_ACCEL_FF_LIMIT_DEG);
}

/**
  * @brief 在 TIM6 的 10 ms 周期中执行任务 6 启动帧目标保持。
  * @details 每个新合法帧只累计一次，前 5 帧位置取四舍五入平均值并锁存。采样完成
  *          前保持平台水平且清除 PID 历史，避免旧目标驱动车辆；锁存后持续执行与
  *          任务 5 相同的钢球 PID。函数只执行定长运算，不阻塞、不打印、不分配内存。
  */
void My_ball_balance_update_start_target_10ms_My(void)
{
  if (My_ball_start_target_ready_My != 0U)
  {
    /* 目标锁存后不再随摄像头漂移，停车阶段也持续补偿钢球扰动和静差。 */
    My_ball_balance_update_target_10ms_My(
      My_ball_balance_debug_My.target_position_mm,
      &My_ball_zero_pid_My,
      &My_ball_zero_correction_My,
      MY_BALL_TASK56_ACCEL_FF_LIMIT_DEG);
    return;
  }

  /*
   * 采样阶段禁止使用旧任务目标。清除 PID 并按既有角度变化率回到水平，既避免
   * 确认瞬间角度阶跃，也保证摄像头无数据时不会驱动钢球。全部状态只由本中断写入。
   */
  My_ball_balance_debug_My.data_fresh = 0U;
  My_ball_balance_debug_My.position_speed_mm_s = 0.0f;
  My_ball_balance_debug_My.measured_speed_mm_s = 0.0f;
  My_ball_balance_debug_My.position_speed_ready = 0U;
  My_ball_balance_debug_My.distance_to_target_mm = 0.0f;
  My_ball_balance_debug_My.control_state = MY_BALL_CONTROL_STATE_HOLD;
  My_pid_reset_My(&My_ball_zero_pid_My);
  My_ball_balance_debug_My.pid_p_angle_deg = 0.0f;
  My_ball_balance_debug_My.pid_i_angle_deg = 0.0f;
  My_ball_balance_debug_My.pid_d_angle_deg = 0.0f;
  My_ball_balance_debug_My.pid_output_angle_deg = 0.0f;
  My_ball_zero_correction_My = My_ball_move_toward_My(
    My_ball_zero_correction_My,
    0.0f,
    MY_BALL_ZERO_CORRECTION_STEP_DEG);
  My_ball_balance_debug_My.correction_angle_deg =
    My_ball_zero_correction_My;
  My_ball_balance_debug_My.target_motor_angle_deg =
    MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG + My_ball_zero_correction_My;
  My_steering_set_target_angle_My(
    My_ball_balance_debug_My.target_motor_angle_deg);
}
