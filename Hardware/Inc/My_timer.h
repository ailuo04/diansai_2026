#ifndef MY_TIMER_H
#define MY_TIMER_H /* 防止任务计时接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief 清零并重新开始计时。
  * @note 重复调用会丢弃上一次结果，并从本次调用时刻重新计时。
  */
void My_timer_start_My(void);

/**
  * @brief 停止计时并冻结当前结果。
  * @note 未开始或已经停止时调用不会改变结果。
  */
void My_timer_stop_My(void);

/**
  * @brief 获取本次计时经过的时间。
  * @retval 经过的毫秒数；确认任务前未开始时返回 0，停止后返回冻结值
  */
uint32_t My_timer_get_elapsed_ms_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_TIMER_H */
