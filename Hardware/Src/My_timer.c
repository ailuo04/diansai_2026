#include "My_timer.h"
#include "stm32f4xx_hal.h"

static volatile uint32_t My_timer_start_tick_My; /* 本次计时的开始时刻，由主循环写入。 */
static volatile uint32_t My_timer_stop_tick_My;  /* 停车时刻，可由 TIM6 中断写入。 */
static volatile uint8_t My_timer_running_My;     /* 主循环读取、TIM6 中断清零的运行标志。 */
static volatile uint8_t My_timer_started_My;     /* 非零表示已经建立过有效计时起点。 */

/**
  * @brief 清零并重新开始计时。
  * @note 使用 HAL 的 1 ms 系统时基，重复调用会重新计时。
  */
void My_timer_start_My(void)
{
  My_timer_start_tick_My = HAL_GetTick();
  My_timer_stop_tick_My = My_timer_start_tick_My;
  My_timer_started_My = 1U;
  My_timer_running_My = 1U;
}

/**
  * @brief 停止计时并冻结当前结果。
  * @details 可在 TIM6 中断中调用，仅执行系统时基读取和定长状态写入，不阻塞、
  *          不分配动态内存，也不访问慢速外设。先写停止时刻再清运行标志，供
  *          主循环在极短临界区内取得一致快照。
  */
void My_timer_stop_My(void)
{
  if (My_timer_running_My != 0U)
  {
    My_timer_stop_tick_My = HAL_GetTick();
    My_timer_running_My = 0U;
  }
}

/**
  * @brief 获取本次计时经过的毫秒数。
  * @retval 正在计时时返回开始至当前的毫秒数，停止后返回冻结值
  */
uint32_t My_timer_get_elapsed_ms_My(void)
{
  uint32_t interrupt_mask = __get_PRIMASK(); /* 保存调用前的全局中断屏蔽状态。 */
  uint32_t elapsed_ms; /* 临界区内形成的完整计时快照。 */

  /*
   * 停车时刻由 TIM6 中断写入。短暂屏蔽中断后一次性读取起点、终点和运行标志，
   * 防止停车瞬间混用更新前后的字段；Cortex-M4 的 32 位读写原子，但多个字段的
   * 组合并不天然原子。临界区不包含格式化、显示或其他耗时操作。
   */
  __disable_irq();
  if (My_timer_started_My == 0U)
  {
    elapsed_ms = 0U;
  }
  else if (My_timer_running_My != 0U)
  {
    elapsed_ms = HAL_GetTick() - My_timer_start_tick_My;
  }
  else
  {
    elapsed_ms = My_timer_stop_tick_My - My_timer_start_tick_My;
  }

  if (interrupt_mask == 0U)
  {
    __enable_irq();
  }

  return elapsed_ms;
}
