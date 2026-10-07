#ifndef MY_ENCODER_H
#define MY_ENCODER_H /* 防止编码器接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdint.h>

/*
 * MG513 GMR 编码器按电机轴每圈 500 线配置；TIM1 使用 TI12 正交模式，
 * 对 A/B 两相进行四倍频计数。30:1 为当前减速箱标称减速比，因此理论上
 * 减速箱输出轴每圈产生 60000 个计数。实际齿轮比和厂家“线数”口径必须
 * 通过输出轴整圈试验复核，若实测不符应以实测值替换本组参数。
 */
/* 电机轴每转的编码器标称线数，不含正交四倍频。 */
#define MY_ENCODER_LINES_PER_MOTOR_REV       500L
/* TIM1 对编码器 A/B 两相边沿进行正交四倍频计数的倍率。 */
#define MY_ENCODER_QUADRATURE_MULTIPLIER       4L
/* 电机轴到减速箱输出轴的标称减速比。 */
#define MY_ENCODER_REDUCTION_RATIO            30L
/* 电机轴每转对应的正交计数，由标称线数和倍频倍率计算。 */
#define MY_ENCODER_COUNTS_PER_MOTOR_REV      \
  (MY_ENCODER_LINES_PER_MOTOR_REV * MY_ENCODER_QUADRATURE_MULTIPLIER)
/* 减速箱输出轴每转对应的理论编码器累计计数。 */
#define MY_ENCODER_COUNTS_PER_OUTPUT_REV     \
  (MY_ENCODER_COUNTS_PER_MOTOR_REV * MY_ENCODER_REDUCTION_RATIO)
/* 减速箱输出轴每度对应的理论编码器计数，供角度与计数换算使用。 */
#define MY_ENCODER_COUNTS_PER_OUTPUT_DEGREE  \
  ((float)MY_ENCODER_COUNTS_PER_OUTPUT_REV / 360.0f)

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

extern volatile My_encoder_debug_t My_encoder_debug_My; /* TIM6 中断更新的编码器调试状态，主循环和调试器只读。 */

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

/**
  * @brief 读取已消除 TIM1 16 位回绕影响的软件累计计数。
  * @retval 当前累计计数；正负方向由编码器 A/B 相接线决定
  * @note Cortex-M4 对自然对齐的 32 位字段单次读取具有原子性。
  */
int32_t My_encoder_get_total_count_My(void);

/**
  * @brief 将编码器累计计数换算为减速箱输出轴相对角度。
  * @param count 相对参考零点的编码器计数
  * @retval 减速箱输出轴相对角度，单位为度
  * @note 该换算不包含曲柄连杆几何关系，不能直接视为平台倾角。
  */
float My_encoder_count_to_output_angle_My(int32_t count);

/**
  * @brief 将减速箱输出轴相对角度换算为编码器计数。
  * @param angle_deg 减速箱输出轴相对角度，单位为度
  * @retval 四舍五入后的相对编码器计数
  * @note 该换算使用标称 500 线和 30:1 减速比，实测后应校正宏参数。
  */
int32_t My_encoder_output_angle_to_count_My(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* MY_ENCODER_H */
