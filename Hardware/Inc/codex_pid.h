#ifndef CODEX_PID_H
#define CODEX_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float kp;
  float ki;
  float kd;
  float error;
  float last_error;
  float prev_error;
  float integral;
  float output;
  float integral_limit;
  float output_limit;
} codex_pid_t;

/**
  * @brief 初始化 PID 控制器。
  * @param pid PID 结构体指针
  * @param kp 比例系数
  * @param ki 积分系数
  * @param kd 微分系数
  * @param integral_limit 积分限幅；小于等于 0 时不限制
  * @param output_limit 输出限幅；小于等于 0 时不限制
  */
void codex_pid_init_codex(volatile codex_pid_t *pid,
                          float kp,
                          float ki,
                          float kd,
                          float integral_limit,
                          float output_limit);

/**
  * @brief 复位 PID 历史状态，保留当前参数。
  * @param pid PID 结构体指针
  */
void codex_pid_reset_codex(volatile codex_pid_t *pid);

/**
  * @brief 位置式 PID 计算。
  * @param pid PID 结构体指针
  * @param target 目标值
  * @param actual 实际值
  * @retval 本次 PID 输出
  */
float codex_pid_calc_position_codex(volatile codex_pid_t *pid, float target, float actual);

/**
  * @brief 角度 PID 计算，误差自动折算到 -180 到 180 度。
  * @param pid PID 结构体指针
  * @param target 目标角度，单位度
  * @param actual 实际角度，单位度
  * @retval 本次 PID 输出
  */
float codex_pid_calc_angle_codex(volatile codex_pid_t *pid, float target, float actual);

/**
  * @brief 增量式 PID 计算，适合速度闭环。
  * @param pid PID 结构体指针
  * @param target 目标值
  * @param actual 实际值
  * @param target_limit 目标限幅；小于等于 0 时不限制
  * @retval 本次 PID 输出
  */
float codex_pid_calc_incremental_codex(volatile codex_pid_t *pid,
                                       float target,
                                       float actual,
                                       float target_limit);

#ifdef __cplusplus
}
#endif

#endif /* CODEX_PID_H */
