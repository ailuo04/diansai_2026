#include "My_key.h"

#define MY_KEY_DEBOUNCE_MS 20U    /* 原始按键电平持续稳定后才确认状态变化的消抖时间，单位毫秒。 */
#define MY_KEY_LONG_PRESS_MS 1000U /* 按下持续达到该时间后在松手时判定为长按，单位毫秒。 */

static GPIO_PinState My_key_raw_state_My; /* 最近一次直接采样到的按键原始电平。 */
static GPIO_PinState My_key_stable_state_My; /* 已通过消抖确认的稳定按键电平。 */
static uint32_t My_key_raw_change_tick_My; /* 原始电平最近一次变化的系统毫秒时刻。 */
static uint32_t My_key_press_tick_My; /* 稳定按下状态建立时的系统毫秒时刻。 */

/**
  * @brief 初始化低电平有效按键的消抖和计时状态。
  */
void My_key_init_My(void)
{
  uint32_t current_tick = HAL_GetTick(); /* 初始化按键状态时读取的统一系统时刻。 */

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
  uint32_t current_tick = HAL_GetTick(); /* 本轮扫描用于消抖和长按判断的系统时刻。 */
  GPIO_PinState sampled_state = HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin); /* 本轮直接读取的低电平有效按键状态。 */
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
