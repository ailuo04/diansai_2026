#include "My_key.h"

#define MY_KEY_DEBOUNCE_MS 20U
#define MY_KEY_LONG_PRESS_MS 1000U

static GPIO_PinState My_key_raw_state_My;
static GPIO_PinState My_key_stable_state_My;
static uint32_t My_key_raw_change_tick_My;
static uint32_t My_key_press_tick_My;

/**
  * @brief 初始化低电平有效按键的消抖和计时状态。
  */
void My_key_init_My(void)
{
  uint32_t current_tick = HAL_GetTick();

  My_key_raw_state_My = HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin);
  My_key_stable_state_My = My_key_raw_state_My;
  My_key_raw_change_tick_My = current_tick;
  My_key_press_tick_My = current_tick;
}

/**
  * @brief 扫描低电平有效按键，在松手后区分短按和长按事件。
  * @retval 本次扫描产生的按键事件
  */
My_key_event_t My_key_scan_My(void)
{
  uint32_t current_tick = HAL_GetTick();
  GPIO_PinState sampled_state = HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin);
  uint32_t press_duration_ms; /* 本次按压从稳定按下到稳定松手的持续时间。 */

  if (sampled_state != My_key_raw_state_My)
  {
    My_key_raw_state_My = sampled_state;
    My_key_raw_change_tick_My = current_tick;
  }

  if (My_key_raw_state_My != My_key_stable_state_My &&
      (uint32_t)(current_tick - My_key_raw_change_tick_My) >= MY_KEY_DEBOUNCE_MS)
  {
    My_key_stable_state_My = My_key_raw_state_My;
    if (My_key_stable_state_My == GPIO_PIN_RESET)
    {
      My_key_press_tick_My = current_tick;
    }
    else
    {
      /* 仅在释放消抖完成后，根据按压时间产生一次事件。 */
      press_duration_ms = (uint32_t)(current_tick - My_key_press_tick_My);
      return press_duration_ms >= MY_KEY_LONG_PRESS_MS ?
        MY_KEY_EVENT_LONG_PRESS : MY_KEY_EVENT_SHORT_PRESS;
    }
  }

  return MY_KEY_EVENT_NONE;
}
