#include "My_app.h"

#include "HWT101.h"
#include "My_ball_balance.h"
#include "My_encoder.h"
#include "My_gray.h"
#include "My_key.h"
#include "My_move.h"
#include "My_oled.h"
#include "My_steering.h"
#include "My_timer.h"
#include "My_uart.h"
#include "tim.h"

#include <stdio.h>

#define MY_APP_GRAY_TRACKING_TASK 2U /* 任务 2 固定执行灰度循迹，其他任务保持停车。 */
#define MY_APP_BALL_BALANCE_TASK  3U /* 任务 3 长按确认后启用钢球位置速度外环。 */
#define MY_APP_S_CURVE_ZERO_TASK   4U /* 任务 4 执行 5 s 底盘 S 型速度曲线，并持续保持钢球 0 点。 */
#define MY_APP_STABLE_TRACKING_TASK 5U /* 任务 5 复用任务 2 路线与终点判定，并使用 S 型起停保持钢球 0 点。 */

#define MY_APP_TASK4_CONTROL_PERIOD_MS 10U   /* 任务 4 与 TIM6 一致的固定控制周期。 */
#define MY_APP_TASK4_RUN_TIME_MS       8000U /* 底盘从启动到完全停车的总时长，包含加速和减速。 */
#define MY_APP_TASK4_ACCEL_TIME_MS     4000U /* 五次多项式 S 型加速段时长。 */
#define MY_APP_TASK4_DECEL_TIME_MS     4000U /* 五次多项式 S 型减速段时长。 */
#define MY_APP_TASK4_CRUISE_PWM         300  /* 加速完成后的直行峰值 PWM，现场可按负载整定。 */

typedef struct
{
  uint8_t selected_task;       /* 当前待确认任务号，仅由主循环读写。 */
  uint8_t task_confirmed;      /* 非零表示任务已经锁存，复位前不再接受短按切换。 */
  char gray_text[9];           /* 8 路灰度状态字符串及结尾空字符。 */
} My_app_context_t;

static My_app_context_t My_app_context_My; /* 主循环独占的任务选择、确认状态和灰度显示缓存。 */
static volatile uint8_t My_app_ball_balance_enabled_My; /* TIM6 读取的任务 3 钢球平衡使能标志。 */
static volatile uint8_t My_app_task4_running_My; /* 非零表示任务 4 的 5 s 底盘 S 曲线仍在执行。 */
static volatile uint8_t My_app_zero_balance_enabled_My; /* 非零表示任务 4 或任务 5 已启用钢球 0 点保持。 */
static volatile uint32_t My_app_task4_elapsed_ms_My; /* TIM6 独占更新的任务 4 底盘运动累计时间。 */

/**
  * @brief 初始化应用层的主循环私有状态。
  * @note 本函数在中断启动前调用，无需临界区保护。
  */
static void My_app_reset_context_My(void)
{
  uint8_t index; /* 当前清零的灰度字符串位置。 */

  My_app_context_My.selected_task = MY_KEY_VALUE_MIN;
  My_app_context_My.task_confirmed = 0U;
  My_app_ball_balance_enabled_My = 0U;
  My_app_task4_running_My = 0U;
  My_app_zero_balance_enabled_My = 0U;
  My_app_task4_elapsed_ms_My = 0U;
  for (index = 0U; index < 8U; index++)
  {
    My_app_context_My.gray_text[index] = '0';
  }
  My_app_context_My.gray_text[8] = '\0';
}

/**
  * @brief 把灰度位图转换为从左到右的显示字符串。
  * @param gray_value 位 7~0 依次对应第 1~8 路灰度传感器
  */
static void My_app_format_gray_My(uint8_t gray_value)
{
  uint8_t index; /* 当前转换的传感器索引。 */

  for (index = 0U; index < 8U; index++)
  {
    My_app_context_My.gray_text[index] =
      (gray_value & (uint8_t)(MY_GRAY_1_MASK >> index)) != 0U ? '1' : '0';
  }
}

/**
  * @brief 计算五次多项式 S 曲线的归一化进度。
  * @param progress 归一化时间进度，范围 0～1
  * @retval 0～1 的平滑速度比例；曲线两端的一阶、二阶导数均为零
  * @details 本函数只执行固定次数浮点运算，由 TIM6 中断路径调用，不阻塞、不分配内存。
  */
static float My_app_task4_s_curve_My(float progress)
{
  if (progress <= 0.0f)
  {
    return 0.0f;
  }
  if (progress >= 1.0f)
  {
    return 1.0f;
  }

  return progress * progress * progress *
    (10.0f + progress * (-15.0f + 6.0f * progress));
}

/**
  * @brief 推进一次任务 4 的 5 s 底盘 S 型加减速灰度循迹状态机。
  * @details 仅由 TIM6 每 10 ms 调用。前 1.5 s 平滑加速，中间 2 s 保持峰值，后
  *          1.5 s 平滑减速；当前速度作为灰度循迹基础 PWM，转向由 Gray_4～Gray_8
  *          的位置误差闭环修正。累计达到 5 s 后立即清零四轮输出并冻结任务计时。
  *          钢球 0 点外环由任务确认后立即并行执行，本函数只读取 GPIO 灰度输入，
  *          不访问慢速外设，也不执行等待或打印。
  */
static void My_app_task4_update_10ms_My(void)
{
  uint32_t elapsed_ms = My_app_task4_elapsed_ms_My; /* 本周期开始时的任务 4 运动时间快照。 */
  float speed_ratio; /* 当前加速、匀速或减速阶段对应的 0～1 速度比例。 */
  float drive_pwm;   /* 本周期沿车体正方向下发给四轮的浮点 PWM。 */

  if (elapsed_ms >= MY_APP_TASK4_RUN_TIME_MS)
  {
    /*
     * 5 s 边界清零底盘目标、速度 PID 和四路 PWM；钢球 0 点外环继续保持开启，
     * 用于停车后收敛和抵抗平台残余振动。共享字段均由本 TIM6 中断独占更新。
     */
    My_gray_pid_stop_My();
    My_app_task4_running_My = 0U;
    My_timer_stop_My();
    return;
  }

  if (elapsed_ms < MY_APP_TASK4_ACCEL_TIME_MS)
  {
    speed_ratio = My_app_task4_s_curve_My(
      (float)elapsed_ms / (float)MY_APP_TASK4_ACCEL_TIME_MS);
  }
  else if (elapsed_ms <
           (MY_APP_TASK4_RUN_TIME_MS - MY_APP_TASK4_DECEL_TIME_MS))
  {
    speed_ratio = 1.0f;
  }
  else
  {
    float decel_progress = (float)(elapsed_ms -
      (MY_APP_TASK4_RUN_TIME_MS - MY_APP_TASK4_DECEL_TIME_MS)) /
      (float)MY_APP_TASK4_DECEL_TIME_MS; /* 减速段从 0 递增到 1 的归一化时间。 */

    speed_ratio = 1.0f - My_app_task4_s_curve_My(decel_progress);
  }

  drive_pwm = (float)MY_APP_TASK4_CRUISE_PWM * speed_ratio;
  /*
   * 任务 4 的纵向速度仍由 5 s S 曲线约束，横向偏差则由灰度循迹 PID 修正；该接口
   * 不启用任务 2 的停止线识别、13 秒降速或反向制动，保证任务 4 只由 5 s 状态机结束。
   */
  My_gray_pid_update_external_pwm_My(drive_pwm);

  /* 累计量只在本中断更新，不与主循环进行读改写共享；末周期饱和到精确 5000 ms。 */
  if (elapsed_ms <=
      (MY_APP_TASK4_RUN_TIME_MS - MY_APP_TASK4_CONTROL_PERIOD_MS))
  {
    My_app_task4_elapsed_ms_My = elapsed_ms + MY_APP_TASK4_CONTROL_PERIOD_MS;
  }
  else
  {
    My_app_task4_elapsed_ms_My = MY_APP_TASK4_RUN_TIME_MS;
  }
}

/**
  * @brief 根据已确认的任务号原子地切换周期控制状态。
  * @details 灰度控制结构、钢球平衡使能和任务 4/5 阶段状态均由 TIM6 中断读取，
  *          因此切换任务时短暂屏蔽中断；临界区只包含常数时间的状态赋值，
  *          不访问阻塞外设。
  */
static void My_app_apply_task_My(void)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 保存调用前的全局中断屏蔽状态。 */

  __disable_irq();
#if MY_APP_SHADOW_DEBUG_MODE != 0U
  /*
   * 影子调试禁止底盘任务继承或启动四轮执行器。灰度停止接口会同步清零四轮
   * 目标、速度 PID 和 PWM；平台倾角电机保持正常位置闭环，供钢球控制器实时
   * 输出补偿角度。
   */
  My_gray_pid_stop_My();
#else
  if (My_app_context_My.selected_task == MY_APP_GRAY_TRACKING_TASK)
  {
    My_gray_pid_start_My();
  }
  else
  {
    /* 非任务 2 必须显式请求底盘停车，避免沿用灰度循迹阶段的控制器状态。 */
    My_gray_pid_stop_My();
  }
#endif

  My_app_task4_running_My = 0U;
  My_app_zero_balance_enabled_My = 0U;
  My_app_task4_elapsed_ms_My = 0U;

  if (My_app_context_My.selected_task == MY_APP_BALL_BALANCE_TASK)
  {
    /* 任务 3 每次确认都从 +50 mm 第一阶段重新开始，并切入持续微调的转向平衡模式。 */
    My_ball_balance_reset_control_My();
    My_steering_set_balance_mode_My(1U);
  }
  else if (My_app_context_My.selected_task == MY_APP_S_CURVE_ZERO_TASK)
  {
    /*
     * 任务 4 确认时先把钢球目标置为 0 mm，并立即开放 0 点外环；底盘 5 s S 型
     * 加减速作为灰度循迹基础速度，与钢球保持并行执行，停车后继续保持 0 点。
    */
    My_ball_balance_reset_zero_control_My();
#if MY_APP_SHADOW_DEBUG_MODE == 0U
    My_gray_pid_start_external_My();
#endif
    My_steering_set_balance_mode_My(1U);
    My_steering_set_target_angle_My(MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG);
#if MY_APP_SHADOW_DEBUG_MODE == 0U
    My_app_task4_running_My = 1U;
#endif
    My_app_zero_balance_enabled_My = 1U;
  }
  else if (My_app_context_My.selected_task == MY_APP_STABLE_TRACKING_TASK)
  {
    /*
     * 任务 5 复用任务 2 的 Gray_4～Gray_8 循迹、三连黑终点确认和最终短路制动，
     * 但使用更低基础速度以及独立 S 型减速，降低底盘纵向冲击对钢球的扰动。钢球
     * 目标从确认时起固定为 0 mm，底盘停车后仍持续运行 0 点外环。
     */
    My_ball_balance_reset_zero_control_My();
#if MY_APP_SHADOW_DEBUG_MODE == 0U
    My_gray_pid_start_stable_My();
#endif
    My_steering_set_balance_mode_My(1U);
    My_steering_set_target_angle_My(MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG);
    My_app_zero_balance_enabled_My = 1U;
  }
  else
  {
    /* 任务 2 以及未实现任务恢复普通定位模式，保留原 0.5°/0.7° 到位断电策略。 */
    My_steering_set_balance_mode_My(0U);
  }

  /*
   * 任务 3 确认后立即开放原分段钢球外环；任务 4 使用独立运行标志，任务 4/5
   * 共用 0 点保持标志。
   * 这些状态均由 TIM6 每 10 ms 读取，必须在同一临界区内提交，避免确认瞬间
   * 读取到一半更新的新任务状态。
   */
  My_app_ball_balance_enabled_My =
    (uint8_t)(My_app_context_My.selected_task == MY_APP_BALL_BALANCE_TASK);
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief 处理按键事件并完成一次性的任务确认。
  * @details 短按仅在未确认时循环任务号；首次长按释放后启动计时，并根据任务号
  *          启停任务 2 循迹、任务 3 钢球控制、任务 4 定时曲线或任务 5 稳定循迹。
  *          任务选择状态只在主循环修改，实际控制使能
  *          通过临界区内的独立标志交给 TIM6 中断。
  */
static void My_app_process_key_My(void)
{
  My_key_event_t key_event = My_key_scan_My(); /* 本轮消抖扫描产生的离散事件。 */

  if (key_event == MY_KEY_EVENT_SHORT_PRESS && My_app_context_My.task_confirmed == 0U)
  {
    My_app_context_My.selected_task++;
    if (My_app_context_My.selected_task > MY_KEY_VALUE_MAX)
    {
      My_app_context_My.selected_task = MY_KEY_VALUE_MIN;
    }
  }
  else if (key_event == MY_KEY_EVENT_LONG_PRESS && My_app_context_My.task_confirmed == 0U)
  {
    /* 先锁存任务和计时起点，再开放对应控制器，保证显示状态先于运动状态确定。 */
    My_app_context_My.task_confirmed = 1U;
    My_timer_start_My();
    My_app_apply_task_My();
  }
}

HAL_StatusTypeDef My_app_init_My(void)
{
  My_app_reset_context_My();

  /* USART3 DMA 队列负责摄像头字节流接收；启动失败时钢球外环没有输入，禁止继续运行。 */
  if (My_uart_init_My() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 软件 I2C 初始化包含传感器上电等待，只允许在中断启动前执行一次。 */
  HWT101_getAngle_Reset();

  /* OLED 不是运动控制前置条件；无应答时记录故障，但保留无屏运行能力。 */
  if (My_oled_init_My() == HAL_OK)
  {
    My_oled_set_cursor_My(0U, 0U);
  }
  else
  {
    printf("OLED NACK: check PF3/PF4, power and address\r\n");
  }

  /* 先启动电机 PWM，再启动独立编码器，保证控制中断开始前反馈链路完整。 */
  if (My_move_init_My() != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (My_encoder_init_My() != HAL_OK)
  {
    My_move_stop_My();
    return HAL_ERROR;
  }
  /* TIM1 反馈启动后再启动对应的 TIM8_CH4 转向 PWM；位置环默认关闭并保持零输出。 */
  if (My_steering_init_My() != HAL_OK)
  {
    My_move_stop_My();
    return HAL_ERROR;
  }
  /* 钢球外环先保持 -79° 水平基准，收到合法摄像头帧后才叠加位置速度修正。 */
  My_ball_balance_init_My();

  /* 装载灰度控制参数；实测黑线为高电平，转向方向按当前底盘接线设置。 */
  My_gray_pid_init_My();
  My_gray_pid_set_parameters_My(70.0f, 0.0f, 10.0f);
  My_gray_pid_set_motion_My(400, 400.0f);
  My_gray_pid_set_active_level_My(1U);
  My_gray_pid_set_steering_direction_My(1);
  My_key_init_My();

  /*
   * 平台倾角电机在正式和影子调试模式下都从固定机械起点建立零点并启动位置环；
   * 影子模式只隔离底盘四轮，不影响钢球闭环对平台倾角的实时控制。
   */
  if (My_steering_start_from_fixed_position_My(-85.0f, 180.0f) != HAL_OK)
  {
    Error_Handler();
  }
#if MY_APP_SHADOW_DEBUG_MODE != 0U
  My_gray_pid_stop_My();
#endif
  /* 最后启动 TIM6，确保编码器内环和钢球位置速度外环看到的状态均已完整初始化。 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    My_steering_stop_My();
    My_move_stop_My();
    return HAL_ERROR;
  }
  return HAL_OK;
}

void My_app_process_My(void)
{
  uint32_t elapsed_ms; /* 本轮界面刷新使用的计时快照。 */
  uint8_t gray_value;  /* 本轮界面刷新使用的 8 路灰度快照。 */

  /* 姿态读取和通信队列轮询均可能访问软件总线或缓冲区，只在主循环执行。 */
  HWT101_getAngle();
  My_uart_poll_My();

  /* 每一显示项使用一次局部快照，避免格式化过程中数据源发生不一致变化。 */
  My_oled_printf_at_My(1U, 1U, "角度：%-7.3f°", Angle);
  elapsed_ms = My_timer_get_elapsed_ms_My();
  My_oled_printf_at_My(2U, 1U, "计时:%5u.%us",
    (unsigned int)(elapsed_ms / 1000U),
    (unsigned int)((elapsed_ms / 100U) % 10U));

  gray_value = My_gray_read_My();
  My_app_format_gray_My(gray_value);
  My_oled_printf_at_My(3U, 1U, "灰度：%s", My_app_context_My.gray_text);

  /* 按键状态机放在显示采集之后；确认产生的运动指令由下一次 TIM6 中断执行。 */
  My_app_process_key_My();
#if MY_APP_SHADOW_DEBUG_MODE != 0U
  My_oled_printf_at_My(4U, 1U, My_app_context_My.task_confirmed != 0U ?
    "任务:%u 调试" : "任务:%u 未确认",
    (unsigned int)My_app_context_My.selected_task);
#else
  My_oled_printf_at_My(4U, 1U, My_app_context_My.task_confirmed != 0U ?
    "任务:%u 已确认" : "任务:%u 未确认",
    (unsigned int)My_app_context_My.selected_task);
#endif
}

void My_app_control_10ms_My(void)
{
  /*
   * 先采集 TIM1 转向编码器和四路底盘编码器周期增量；任务 3 更新原有分段钢球目标，
   * 任务 4 并行推进 5 s 底盘 S 型曲线和 0 点钢球外环，任务 5 则由灰度模块执行
   * 与任务 2 相同的循迹和终点判定，并在停车阶段执行 S 型减速。随后执行转向位置
   * 内环，最后更新任务 2/5 灰度循迹。全部操作均为常数时间，必须显著小于 10 ms。
  */
  My_encoder_update_My();
  My_move_update_encoder_My();
#if MY_APP_SHADOW_DEBUG_MODE != 0U
  /*
   * 影子模式每个 TIM6 周期都强制清零四轮目标、速度 PID 和 PWM，防止后续任务
   * 逻辑调整时意外遗留底盘使能状态。平台倾角位置环不在停机范围内，仍会在本
   * 周期根据钢球外环计算出的目标角正常驱动。停止接口不阻塞、不打印、不分配内存。
   */
  My_gray_pid_stop_My();
#endif
  if (My_app_ball_balance_enabled_My != 0U)
  {
    /* 任务 3 两段均执行位置式 PID，D 项直接使用摄像头速度；+50 mm 到点后切至 -50 mm。 */
    My_ball_balance_update_10ms_My();
  }
  if (My_app_task4_running_My != 0U)
  {
    /* 任务 4 的底盘时间、S 曲线和灰度循迹只由 TIM6 更新，主循环不参与实时速度规划。 */
    My_app_task4_update_10ms_My();
  }
  if (My_app_zero_balance_enabled_My != 0U)
  {
    /* 任务 4/5 在行驶和停车后都直接使用摄像头回传速度，把钢球保持在 0±4 mm。 */
    My_ball_balance_update_zero_10ms_My();
  }
  My_steering_update_10ms_My();
  My_gray_pid_update_My();
}

/**
  * @brief 将 USART3 接收到的字节流送入钢球摄像头协议解析器。
  * @details 本函数由主循环中的 `My_uart_poll_My()` 调用，不运行在中断上下文；
  *          DMA 空闲事件不等于协议帧边界，因此解析器会跨回调保留半帧，也会在
  *          单次回调内连续处理多帧。合法位置和速度整体提交给 TIM6 外环读取。
  * @param data 本次 DMA 空闲事件确认的数据首地址，仅在本次回调期间有效
  * @param length 本次收到的有效字节数，可能不是 13 的整数倍
  */
void My_uart_rx_frame_callback_My(const uint8_t *data, uint16_t length)
{
  /* 只做主循环流式解析，不再把摄像头数据原样发回 USART3。 */
  My_ball_balance_feed_My(data, length);
}
