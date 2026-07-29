#include "My_timer.h"
#include "stm32f4xx_hal.h"

static uint32_t My_timer_start_tick_My; /* 本次计时的开始时刻。 */
static uint32_t My_timer_stop_tick_My;  /* 本次计时的停止时刻。 */
static uint8_t My_timer_running_My;     /* 非零表示当前正在计时。 */

/**
  * @brief 清零并重新开始计时。
  * @note 使用 HAL 的 1 ms 系统时基，重复调用会重新计时。
  */
void My_timer_start_My(void)
{
  My_timer_start_tick_My = HAL_GetTick();
  My_timer_stop_tick_My = My_timer_start_tick_My;
  My_timer_running_My = 1U;
}

/**
  * @brief 停止计时并冻结当前结果。
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
  uint32_t current_tick = My_timer_stop_tick_My; /* 本次计算使用的结束时刻。 */

  if (My_timer_running_My != 0U)
  {
    current_tick = HAL_GetTick();
  }

  return current_tick - My_timer_start_tick_My;
}
