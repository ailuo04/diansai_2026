#ifndef MY_STEERING_H
#define MY_STEERING_H /* 防止转向位置环接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "My_pid.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define MY_STEERING_PWM_MAX 999 /* TIM8 周期为 999，对应允许写入的最大比较值。 */

/*
 * 到位后使用带滞回的位置死区：误差进入 0.5 度时清除 PID 并关闭驱动，只有
 * 负载使误差扩大到 0.7 度之外才恢复闭环。两个阈值的间隔可避免编码器量化
 * 误差或机构微振在边界附近造成电机频繁启停。
 */
#define MY_STEERING_STOP_DEADBAND_DEG   0.5f /* 位置误差进入该角度范围后关闭电机，单位为度。 */
#define MY_STEERING_RESUME_DEADBAND_DEG 0.7f /* 到位保持后误差超过该角度才恢复闭环，单位为度。 */

/* 钢球平衡使用更小的停止死区，死区外直接采用编码器位置 PID 的实际输出。 */
#define MY_STEERING_BALANCE_DEADBAND_DEG          0.15f /* 平衡模式允许的电机输出轴微小误差，单位为度。 */

/**
  * @brief TIM1 编码器转向电机的位置环运行状态。
  * @details 本结构主要供 Keil Watch 观察。除参数设置接口外，各运行字段由
  *          TIM6 的 10 ms 中断更新；主循环连续读取多个字段时不构成同步快照。
  */
typedef struct
{
  My_pid_t pid;              /**< 以编码器累计计数为输入的位置式 PID。 */
  int32_t zero_count;        /**< 本次人工设定的减速箱输出轴相对零点。 */
  int32_t min_target_count;  /**< 允许下发的位置目标下限。 */
  int32_t max_target_count;  /**< 允许下发的位置目标上限。 */
  int32_t target_count;      /**< 目标位置，单位为 TIM1 编码器累计计数。 */
  int32_t actual_count;      /**< 最近控制周期读取的实际累计计数。 */
  int32_t previous_actual_count; /**< 上一控制周期累计计数，平衡模式用于计算测量速度。 */
  float target_angle_deg;    /**< 相对 zero_count 的目标输出轴角度，单位为度。 */
  float actual_angle_deg;    /**< 相对 zero_count 的实际输出轴角度，单位为度。 */
  float angle_limit_deg;     /**< 当前输出轴目标角度软限位的绝对值。 */
  int16_t pwm_output;        /**< 最近输出，正负表示方向，绝对值范围 0～999。 */
  int32_t actual_delta_count; /**< 最近 10 ms 输出轴编码器累计计数变化量。 */
  int8_t motor_direction;    /**< 电机方向映射，1 为默认接线，-1 为反向。 */
  uint8_t enabled;           /**< 非零表示位置环允许驱动电机。 */
  uint8_t zero_ready;        /**< 非零表示已通过固定起点或回零流程建立可信相对零点。 */
  uint8_t in_deadband;       /**< 非零表示已进入到位死区，PID 积分和电机输出保持为零。 */
  uint8_t balance_mode;      /**< 非零表示使用钢球平衡专用微死区并直接输出位置 PID。 */
} My_steering_control_t;

extern volatile My_steering_control_t My_steering_control_My; /* TIM6 中断更新的转向位置环状态。 */

/**
  * @brief 初始化 TIM1 编码器对应的转向电机位置环并启动 TIM8_CH4 PWM。
  * @details 调用前必须完成 GPIO、TIM1、TIM8 和编码器初始化。初始化后目标为
  *          当前编码器位置，位置环保持关闭，PWM 为零，零点和活动范围均未就绪，
  *          因此不会在上电时把未知机械位置误认为水平位置并主动驱动电机。
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
  * @brief 切换普通定位模式与钢球平衡专用模式。
  * @param enabled 非零启用平衡模式，零恢复普通定位模式
  * @details 模式切换会清除 PID 历史和到位状态，但不会改变当前位置、目标角度、
  *          软件限位或使能状态。调用过程使用短临界区，可由主循环任务切换逻辑调用。
  */
void My_steering_set_balance_mode_My(uint8_t enabled);

/**
  * @brief 以当前位置建立减速箱输出轴相对零点并停止位置环。
  * @details 本函数不清零 TIM1 软件累计计数，只更新本电机控制器的相对零点；
  *          同时关闭 PWM、清除 PID 历史并锁定目标范围。应仅在输出轴已经由
  *          人工或机械结构放到可重复的固定起点，或自动回零流程已经到达标定
  *          位置时调用；调用后还必须设置实际允许角度，位置环才能启动。
  */
void My_steering_set_zero_My(void);

/**
  * @brief 设置减速箱输出轴相对零点的目标角度软限位。
  * @param limit_deg 正负对称限位的绝对值，单位为度；必须大于 0
  * @retval HAL_OK 表示设置成功，HAL_ERROR 表示参数非法
  * @note 这是软件目标限位，不能代替机械限位开关或实体限位块。
  */
HAL_StatusTypeDef My_steering_set_angle_limit_My(float limit_deg);

/**
  * @brief 启动位置环并运动到指定编码器累计计数。
  * @param target_count 目标位置，单位为 TIM1 编码器累计计数
  * @retval HAL_OK 表示位置环已启动，HAL_ERROR 表示零点或角度限位尚未设置
  * @note 启动时清除 PID 历史状态，避免沿用上一次控制的积分和微分量。
  */
HAL_StatusTypeDef My_steering_start_My(int32_t target_count);

/**
  * @brief 在线更新目标位置，不清除 PID 历史状态。
  * @param target_count 目标位置，单位为 TIM1 编码器累计计数
  */
void My_steering_set_target_My(int32_t target_count);

/**
  * @brief 启动位置环并运动到指定减速箱输出轴相对角度。
  * @param target_angle_deg 相对人工零点的目标角度，单位为度
  * @retval HAL_OK 表示位置环已启动，HAL_ERROR 表示零点或角度限位尚未设置
  * @note 目标会被当前角度软限位约束；此角度不是连杆驱动后的平台倾角。
  */
HAL_StatusTypeDef My_steering_start_angle_My(float target_angle_deg);

/**
  * @brief 从上电时的固定机械起点运动到标定目标角度。
  * @param target_angle_deg 相对固定起点的目标输出轴角度，单位为度
  * @param limit_deg 相对固定起点的正负对称安全限位，单位为度；必须大于目标绝对值
  * @retval HAL_OK 表示位置环已启动，HAL_ERROR 表示参数非法或启动失败
  * @details 函数依次把当前位置记录为零点、设置安全限位并启动位置环。该流程
  *          只适用于每次上电起始位置都能机械重复的场景；若起始位置放错，增量
  *          编码器无法识别，平台最终位置也会随之偏移。
  */
HAL_StatusTypeDef My_steering_start_from_fixed_position_My(float target_angle_deg,
                                                          float limit_deg);

/**
  * @brief 在线更新减速箱输出轴目标角度，不清除 PID 历史状态。
  * @param target_angle_deg 相对人工零点的目标角度，单位为度
  * @note 目标会被当前角度软限位约束；此角度不是连杆驱动后的平台倾角。
  */
void My_steering_set_target_angle_My(float target_angle_deg);

/**
  * @brief 读取最近控制周期计算出的减速箱输出轴相对角度。
  * @retval 相对人工零点的输出轴角度，单位为度
  */
float My_steering_get_actual_angle_My(void);

/**
  * @brief 停止位置环并立即关闭 PWM。
  * @details 同时清除 PID 状态，并把目标锁定为当前实际位置，避免再次启动时跳变。
  */
void My_steering_stop_My(void);

/**
  * @brief 执行一次转向电机位置环计算。
  * @details 固定在 TIM1 编码器完成本周期采样后，由 TIM6 的 10 ms 中断调用。
  *          到位后清除 PID 积分并关闭驱动，位置误差超过恢复阈值后重新闭环。
  *          函数只执行常数时间计算和 GPIO/PWM 写入，不得在其中加入阻塞操作。
  */
void My_steering_update_10ms_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_STEERING_H */
