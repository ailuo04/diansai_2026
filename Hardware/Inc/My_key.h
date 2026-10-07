#ifndef MY_KEY_H
#define MY_KEY_H /* 防止按键接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define MY_KEY_VALUE_MIN 2U /* 任务选择允许显示和确认的最小任务号。 */
#define MY_KEY_VALUE_MAX 6U /* 任务选择允许显示和确认的最大任务号。 */

typedef enum
{
  MY_KEY_EVENT_NONE = 0U,
  MY_KEY_EVENT_SHORT_PRESS,
  MY_KEY_EVENT_LONG_PRESS
} My_key_event_t;

/**
  * @brief 初始化低电平有效按键的消抖和计时状态。
  */
void My_key_init_My(void);

/**
  * @brief 扫描低电平有效按键，在松手后区分短按和长按事件。
  * @retval 本次扫描产生的按键事件
  */
My_key_event_t My_key_scan_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_KEY_H */
