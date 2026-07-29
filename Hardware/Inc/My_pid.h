#ifndef MY_PID_H
#define MY_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float kp;             /* 比例系数，用于放大当前误差。 */
  float ki;             /* 积分系数，用于消除长期静态误差。 */
  float kd;             /* 微分系数，用于抑制误差变化过快。 */
  float error;          /* 当前控制周期误差。 */
  float last_error;     /* 上一个控制周期误差。 */
  float prev_error;     /* 上上个控制周期误差，增量式 PID 微分项使用。 */
  float integral;       /* 位置式 PID 的积分累计值。 */
  float output;         /* 当前 PID 输出值。 */
  float integral_limit; /* 积分限幅绝对值，小于等于 0 表示不限幅。 */
  float output_limit;   /* 输出限幅绝对值，小于等于 0 表示不限幅。 */
} My_pid_t;

/**
  * @brief 初始化 PID 控制器。
  * @param pid PID 结构体指针
  * @param kp 比例系数
  * @param ki 积分系数
  * @param kd 微分系数
  * @param integral_limit 积分限幅；小于等于 0 时不限制
  * @param output_limit 输出限幅；小于等于 0 时不限制
  */
void My_pid_init_My(volatile My_pid_t *pid,
                          float kp,
                          float ki,
                          float kd,
                          float integral_limit,
                          float output_limit);

/**
  * @brief 复位 PID 历史状态，保留当前参数。
  * @param pid PID 结构体指针
  */
void My_pid_reset_My(volatile My_pid_t *pid);

/**
  * @brief 位置式 PID 计算。
  * @param pid PID 结构体指针
  * @param target 目标值
  * @param actual 实际值
  * @retval 本次 PID 输出
  */
float My_pid_calc_position_My(volatile My_pid_t *pid, float target, float actual);

/**
  * @brief 角度 PID 计算，误差自动折算到 -180 到 180 度。
  * @param pid PID 结构体指针
  * @param target 目标角度，单位度
  * @param actual 实际角度，单位度
  * @retval 本次 PID 输出
  */
float My_pid_calc_angle_My(volatile My_pid_t *pid, float target, float actual);

/**
  * @brief 增量式 PID 计算，适合速度闭环。
  * @param pid PID 结构体指针
  * @param target 目标值
  * @param actual 实际值
  * @param target_limit 目标限幅；小于等于 0 时不限制
  * @retval 本次 PID 输出
  */
float My_pid_calc_incremental_My(volatile My_pid_t *pid,
                                       float target,
                                       float actual,
                                       float target_limit);

#ifdef __cplusplus
}
#endif

#endif /* MY_PID_H */
