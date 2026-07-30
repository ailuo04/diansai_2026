#ifndef MY_ENCODER_H
#define MY_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
  * @brief 单路 TIM1 编码器调试数据。
  * @details 在 Keil Watch 窗口中添加 My_encoder_debug_My 即可观察全部字段。
  *          各字段由 TIM6 的 10 ms 中断更新，主循环或调试器只读；Cortex-M4
  *          对这些自然对齐的 16/32 位字段可原子访问，但连续读取多个字段时可能
  *          跨越一次采样，不能把它们视为严格同步快照。
  */
typedef struct
{
  int32_t raw_count;     /**< TIM1 当前计数的有符号视图，反向越过零后显示 -1、-2……。 */
  int16_t delta_count;   /**< 最近 10 ms 的有符号计数增量，正负方向由 A/B 相接线决定。 */
  int32_t total_count;   /**< 软件累计计数，已消除 TIM1 的 16 位回绕影响。 */
  uint32_t sample_count; /**< 已完成的 10 ms 采样次数，用于确认中断持续运行。 */
  uint8_t running;       /**< 1 表示 TIM1 编码器接口已经成功启动。 */
} My_encoder_debug_t;

extern volatile My_encoder_debug_t My_encoder_debug_My;

/**
  * @brief 初始化并启动 TIM1 单路正交编码器。
  * @details 调用前必须完成 MX_TIM1_Init()；初始化会清零硬件计数器和调试数据，
  *          然后同时使能 TIM1 CH1 与 CH2。
  * @retval HAL_OK 表示启动成功，HAL_ERROR 表示定时器启动失败
  */
HAL_StatusTypeDef My_encoder_init_My(void);

/**
  * @brief 采样一次 TIM1 编码器并更新调试数据。
  * @details 固定由 TIM6 的 10 ms 中断调用；单周期计数变化绝对值必须小于 32768，
  *          才能通过 16 位差值可靠区分方向并处理计数器回绕。raw_count 是硬件
  *          计数器的有符号显示值，超过单个 16 位量程后的连续位置应读取 total_count。
  */
void My_encoder_update_My(void);

/**
  * @brief 清零 TIM1 硬件计数器和全部软件累计数据。
  * @note 应在编码器停止或外部已进入临界区时调用，避免与 TIM6 采样并发。
  */
void My_encoder_reset_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_ENCODER_H */
