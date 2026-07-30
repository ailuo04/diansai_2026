#ifndef MY_STEERING_H
#define MY_STEERING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "My_pid.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define MY_STEERING_PWM_MAX 999 /* TIM4 周期为 999，对应允许写入的最大比较值。 */

/**
  * @brief TIM1 编码器转向电机的位置环运行状态。
  * @details 本结构主要供 Keil Watch 观察。除参数设置接口外，各运行字段由
  *          TIM6 的 10 ms 中断更新；主循环连续读取多个字段时不构成同步快照。
  */
typedef struct
{
  My_pid_t pid;              /**< 以编码器累计计数为输入的位置式 PID。 */
  int32_t target_count;      /**< 目标位置，单位为 TIM1 编码器累计计数。 */
  int32_t actual_count;      /**< 最近控制周期读取的实际累计计数。 */
  int16_t pwm_output;        /**< 最近输出，正负表示方向，绝对值范围 0～999。 */
  int8_t motor_direction;    /**< 电机方向映射，1 为默认接线，-1 为反向。 */
  uint8_t enabled;           /**< 非零表示位置环允许驱动电机。 */
} My_steering_control_t;

extern volatile My_steering_control_t My_steering_control_My;

/**
  * @brief 初始化 TIM1 编码器对应的转向电机位置环并启动 TIM4_CH3 PWM。
  * @details 调用前必须完成 GPIO、TIM1、TIM4 和编码器初始化。初始化后目标为
  *          当前编码器位置，位置环保持关闭，PWM 为零，不会主动驱动电机。
  * @retval HAL_OK 表示 PWM 启动成功，HAL_ERROR 表示启动失败
  */
HAL_StatusTypeDef My_steering_init_My(void);

/**
  * @brief 设置位置环离散 PID 参数。
  * @param kp 比例系数
  * @param ki 积分系数
  * @param kd 微分系数
  * @param integral_limit 积分累计限幅；小于等于 0 表示不限幅
  * @param output_limit PWM 输出限幅；会强制约束在 1～999
  * @note 参数对应固定 10 ms 采样周期，修改 TIM6 周期后必须重新整定。
  */
void My_steering_set_pid_My(float kp,
                            float ki,
                            float kd,
                            float integral_limit,
                            float output_limit);

/**
  * @brief 设置电机正方向与编码器计数正方向的映射。
  * @param direction 大于等于 0 时使用默认方向，小于 0 时反向
  */
void My_steering_set_direction_My(int8_t direction);

/**
  * @brief 启动位置环并运动到指定编码器累计计数。
  * @param target_count 目标位置，单位为 TIM1 编码器累计计数
  * @note 启动时清除 PID 历史状态，避免沿用上一次控制的积分和微分量。
  */
void My_steering_start_My(int32_t target_count);

/**
  * @brief 在线更新目标位置，不清除 PID 历史状态。
  * @param target_count 目标位置，单位为 TIM1 编码器累计计数
  */
void My_steering_set_target_My(int32_t target_count);

/**
  * @brief 停止位置环并立即关闭 PWM。
  * @details 同时清除 PID 状态，并把目标锁定为当前实际位置，避免再次启动时跳变。
  */
void My_steering_stop_My(void);

/**
  * @brief 执行一次转向电机位置环计算。
  * @details 固定在 TIM1 编码器完成本周期采样后，由 TIM6 的 10 ms 中断调用。
  *          函数只执行常数时间计算和 GPIO/PWM 写入，不得在其中加入阻塞操作。
  */
void My_steering_update_10ms_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_STEERING_H */
