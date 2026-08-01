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
#define MY_MOVE_SPEED_PID_DEFAULT_KP 0.40f   /* 四轮增量式速度 PID 的保守初始比例系数，必须实车整定。 */
#define MY_MOVE_SPEED_PID_DEFAULT_KI 0.02f   /* 四轮增量式速度 PID 的保守初始积分系数，必须实车整定。 */
#define MY_MOVE_SPEED_PID_DEFAULT_KD 0.00f   /* 编码器周期计数量化明显，默认关闭微分项。 */

extern volatile int16_t My_move_debug_velocity_1_My; /* Keil Watch 调试用：1 号轮 10 ms 编码器速度。 */
extern volatile int16_t My_move_debug_velocity_2_My; /* Keil Watch 调试用：2 号轮 10 ms 编码器速度。 */
extern volatile int16_t My_move_debug_velocity_3_My; /* Keil Watch 调试用：3 号轮 10 ms 编码器速度。 */
extern volatile int16_t My_move_debug_velocity_4_My; /* Keil Watch 调试用：4 号轮 10 ms 编码器速度。 */

/**
  * @brief 初始化四路电机 PWM 与四路编码器。
  * @details PWM 使用 TIM5 CH1~CH4；1 号轮使用 PA8/PA9 外部中断软件正交计数，
  *          2 号轮使用 TIM2，3 号轮使用 TIM3，4 号轮使用 TIM4 硬件正交计数。
  *          TIM1 只属于转向位置编码器，TIM8_CH4 只属于转向 PWM。
  * @retval HAL_OK 表示全部启动成功，否则表示至少一个定时器启动失败
  */
HAL_StatusTypeDef My_move_init_My(void);

/**
  * @brief 停止四个电机并清零速度 PID 输出。
  * @details 安全停机直接把四路 PWM 写为零；这是唯一允许绕过速度 PID 的输出路径。
  */
void My_move_stop_My(void);

/**
  * @brief 读取四路编码器本周期增量。
  * @details 应在 TIM6 的 10 ms 周期中调用；函数只刷新本周期速度反馈，不维护底盘
  *          长期位置累计。调用后本周期速度可用 `My_move_get_velocity_My()` 读取。
  */
void My_move_update_encoder_My(void);

/**
  * @brief 清零四路编码器累计值与硬件计数器。
  * @details 当前速度闭环不维护底盘累计位置，本接口主要用于清零周期速度和硬件 CNT。
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
  * @retval 兼容旧接口保留值；当前速度闭环不更新该值，轮号非法时返回 0
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
  * @details 四路非零 PWM 只能由本函数的速度 PID 产生；PID 未初始化时强制输出零，
  *          禁止把速度目标直接解释为 PWM。
  */
void My_move_velocity_pid_update_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_MOVE_H */
