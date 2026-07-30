#include "My_app.h"

#include "HWT101.h"
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

typedef struct
{
  uint8_t selected_task;       /* 当前待确认任务号，仅由主循环读写。 */
  uint8_t task_confirmed;      /* 非零表示任务已经锁存，复位前不再接受短按切换。 */
  char gray_text[9];           /* 8 路灰度状态字符串及结尾空字符。 */
} My_app_context_t;

static My_app_context_t My_app_context_My;

/**
  * @brief 初始化应用层的主循环私有状态。
  * @note 本函数在中断启动前调用，无需临界区保护。
  */
static void My_app_reset_context_My(void)
{
  uint8_t index; /* 当前清零的灰度字符串位置。 */

  My_app_context_My.selected_task = MY_KEY_VALUE_MIN;
  My_app_context_My.task_confirmed = 0U;
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
  * @details 灰度控制结构同时由 TIM6 中断读取，因此更新多个字段时短暂屏蔽中断；
  *          临界区只包含常数时间的状态赋值，不执行外设访问或阻塞操作。
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
    /* 未实现任务必须显式请求停车，避免沿用调试阶段的控制器状态。 */
    My_gray_pid_stop_My();
  }
  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief 处理按键事件并完成一次性的任务确认。
  * @details 短按仅在未确认时循环任务号；首次长按释放后启动计时，并根据任务号
  *          启停灰度循迹。该状态只在主循环访问，不与中断直接共享。
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

  /* USART3 DMA 队列是调试输出和姿态通信的基础，启动失败时禁止继续运行。 */
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

  /* 装载灰度控制参数；实测黑线为高电平，转向方向按当前底盘接线设置。 */
  My_gray_pid_init_My();
  My_gray_pid_set_parameters_My(70.0f, 0.0f, 10.0f);
  My_gray_pid_set_motion_My(400, 400.0f);
  My_gray_pid_set_active_level_My(1U);
  My_gray_pid_set_steering_direction_My(1);
  My_key_init_My();

  /* 最后启动 TIM6，避免周期回调观察到尚未初始化的编码器或控制器状态。 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    My_steering_stop_My();
    My_move_stop_My();
    return HAL_ERROR;
  }

  printf("USART3 DMA ready\r\n");
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
   * 先采集 TIM1 编码器，再把同一周期的累计位置交给转向位置环，最后更新灰度
   * 循迹和四轮输出。全部操作均为常数时间，总执行时间必须显著小于 10 ms。
   */
  My_encoder_update_My();
  My_steering_update_10ms_My();
  My_gray_pid_update_My();
}
