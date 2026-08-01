#ifndef MY_MOVE_H
#define MY_MOVE_H /* 防止四轮运动控制接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "My_pid.h"
#include "main.h"
#include <stdint.h>

#define MY_MOVE_WHEEL_COUNT          4U      /* 底盘轮子数量，数组下标 1~4 对应四个电机。 */
#define MY_MOVE_PWM_MAX              1000    /* 电机 PWM 输出最大绝对值，1000 对应 TIM5 的 100% 占空比。 */
#define MY_MOVE_DEFAULT_TARGET_LIMIT 1500.0f /* 速度闭环默认目标限幅，防止异常目标直接打满输出。 */

/**
  * @brief 初始化四路电机 PWM 与四路编码器。
  * @details PWM 使用 TIM5 CH1~CH4；编码器轮序沿用旧工程：
  *          1 号轮 TIM3，2 号轮 TIM2，3 号轮 TIM1，4 号轮 TIM4。
  * @retval HAL_OK 表示全部启动成功，否则表示至少一个定时器启动失败
  */
HAL_StatusTypeDef My_move_init_My(void);

/**
  * @brief 直接控制四个电机 PWM。
  * @param wheel_1 1 号电机指令，正负表示方向
  * @param wheel_2 2 号电机指令，正负表示方向
  * @param wheel_3 3 号电机指令，正负表示方向
  * @param wheel_4 4 号电机指令，正负表示方向
  */
void My_move_control_My(int16_t wheel_1,
                              int16_t wheel_2,
                              int16_t wheel_3,
                              int16_t wheel_4);

/**
  * @brief 停止四个电机并清零速度 PID 输出。
  */
void My_move_stop_My(void);

/**
  * @brief 读取并累计四路编码器增量。
  * @details 应在固定周期中调用；调用后本周期速度可用
  *          `My_move_get_velocity_My()` 读取。
  */
void My_move_update_encoder_My(void);

/**
  * @brief 清零四路编码器累计值与硬件计数器。
  */
void My_move_reset_encoder_My(void);

/**
  * @brief 读取指定轮子的本周期编码器速度。
  * @param wheel 轮号，范围 1~4
  * @retval 本周期编码器增量；轮号非法时返回 0
  */
int16_t My_move_get_velocity_My(uint8_t wheel);

/**
  * @brief 读取指定轮子的累计编码器位置。
  * @param wheel 轮号，范围 1~4
  * @retval 累计编码器增量；轮号非法时返回 0
  */
int32_t My_move_get_encoder_My(uint8_t wheel);

/**
  * @brief 设置四路速度 PID 参数。
  * @param kp 比例系数
  * @param ki 积分系数
  * @param kd 微分系数
  * @param integral_limit 积分限幅
  * @param output_limit 输出限幅
  */
void My_move_set_speed_pid_My(float kp,
                                    float ki,
                                    float kd,
                                    float integral_limit,
                                    float output_limit);

/**
  * @brief 设置四轮目标速度。
  * @param wheel_1 1 号轮目标
  * @param wheel_2 2 号轮目标
  * @param wheel_3 3 号轮目标
  * @param wheel_4 4 号轮目标
  */
void My_move_set_target_My(float wheel_1,
                                 float wheel_2,
                                 float wheel_3,
                                 float wheel_4);

/**
  * @brief 麦克纳姆轮逆运动学解算并写入四轮目标速度。
  * @param move_vx x 方向速度
  * @param move_vy y 方向速度
  * @param move_vw 旋转速度
  */
void My_move_mecanum_inverse_My(float move_vx, float move_vy, float move_vw);

/**
  * @brief 根据目标速度和编码器反馈执行一次速度闭环。
  * @details 若未设置 PID 参数，则把目标速度直接作为 PWM 指令输出，便于低风险空载调试。
  */
void My_move_velocity_pid_update_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_MOVE_H */
