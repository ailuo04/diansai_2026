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

typedef struct
{
  uint8_t selected_task;       /* 当前待确认任务号，仅由主循环读写。 */
  uint8_t task_confirmed;      /* 非零表示任务已经锁存，复位前不再接受短按切换。 */
  char gray_text[9];           /* 8 路灰度状态字符串及结尾空字符。 */
} My_app_context_t;

static My_app_context_t My_app_context_My; /* 主循环独占的任务选择、确认状态和灰度显示缓存。 */
static volatile uint8_t My_app_ball_balance_enabled_My; /* TIM6 读取的任务 3 钢球平衡使能标志。 */

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
  * @brief 根据已确认的任务号原子地切换周期控制状态。
  * @details 灰度控制结构和钢球平衡使能标志均由 TIM6 中断读取，因此切换任务时
  *          短暂屏蔽中断；临界区只包含常数时间的状态赋值，不访问阻塞外设。
  */
static void My_app_apply_task_My(void)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 保存调用前的全局中断屏蔽状态。 */

  __disable_irq();
  if (My_app_context_My.selected_task == MY_APP_GRAY_TRACKING_TASK)
  {
    My_gray_pid_start_My();
  }
  else
  {
    /* 非任务 2 必须显式请求底盘停车，避免沿用灰度循迹阶段的控制器状态。 */
    My_gray_pid_stop_My();
  }

  if (My_app_context_My.selected_task == MY_APP_BALL_BALANCE_TASK)
  {
    /* 任务 3 每次确认都从 +50 mm 第一阶段重新开始，并切入持续微调的转向平衡模式。 */
    My_ball_balance_reset_control_My();
    My_steering_set_balance_mode_My(1U);
  }
  else
  {
    /* 非钢球任务恢复普通定位模式，保留原 0.5°/0.7° 到位断电策略。 */
    My_steering_set_balance_mode_My(0U);
  }

  /*
   * 只有任务号 3 已经通过长按确认后才进入本函数并开放钢球外环。该标志由
   * TIM6 每 10 ms 读取，必须在同一临界区内提交，避免确认瞬间读取到中间状态。
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
  *          启停灰度循迹或钢球平衡。任务选择状态只在主循环修改，实际控制使能
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
  /* TIM1 反馈启动后再启动对应的 TIM4_CH3 转向 PWM；位置环默认关闭并保持零输出。 */
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

  /* 先把固定机械起点映射到平台水平基准，再开放 10 ms 控制中断产生电机输出。 */
  if (My_steering_start_from_fixed_position_My(-85.0f, 180.0f) != HAL_OK)
  {
    Error_Handler();
  }
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
  My_oled_printf_at_My(4U, 1U, My_app_context_My.task_confirmed != 0U ?
    "任务:%u 已确认" : "任务:%u 未确认",
    (unsigned int)My_app_context_My.selected_task);
}

void My_app_control_10ms_My(void)
{
  /*
   * 先采集 TIM1 编码器；仅当任务 3 已确认时，使用摄像头位置和速度控制钢球先到
   * +50 mm，低速到达 +50 mm 后再转向 -50 mm 并保持静止；随后执行转向位置
   * 内环，最后更新灰度循迹和四轮输出。全部操作均为常数时间，必须显著小于 10 ms。
   */
  My_encoder_update_My();
  if (My_app_ball_balance_enabled_My != 0U)
  {
    /* 任务 3 以 +50 mm 为首目标，低速到点后切至 -50 mm，并按第二段参数提前制动。 */
    My_ball_balance_update_10ms_My();
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
