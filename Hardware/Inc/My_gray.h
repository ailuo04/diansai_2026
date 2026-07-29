#ifndef MY_GRAY_H
#define MY_GRAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "My_pid.h"

#define MY_GRAY_1_MASK (0x80U)
#define MY_GRAY_2_MASK (0x40U)
#define MY_GRAY_3_MASK (0x20U)
#define MY_GRAY_4_MASK (0x10U)
#define MY_GRAY_5_MASK (0x08U)
#define MY_GRAY_6_MASK (0x04U)
#define MY_GRAY_7_MASK (0x02U)
#define MY_GRAY_8_MASK (0x01U)

#define MY_GRAY_PID_DEFAULT_BASE_PWM         250
#define MY_GRAY_PID_DEFAULT_CORRECTION_LIMIT 300.0f
#define MY_GRAY_PID_DEFAULT_ACTIVE_LEVEL     0U

typedef struct
{
  My_pid_t pid;           /* 灰度位置环 PID，Kp、Ki、Kd 初始化为 0。 */
  int16_t base_pwm;       /* 直行基础 PWM。 */
  float correction_limit; /* 左右修正量的绝对值上限。 */
  float error;            /* 当前加权位置误差，范围为 -7 到 7。 */
  float correction;       /* 当前转向修正量。 */
  uint8_t raw_value;      /* 当前 8 路灰度原始位图。 */
  uint8_t line_detected;  /* 非零表示当前至少一路检测到赛道线。 */
  uint8_t active_level;   /* 0 表示低电平检测到线，1 表示高电平检测到线。 */
  int8_t steering_direction; /* 转向方向，接线相反时设为 -1。 */
  uint8_t enabled;        /* 非零表示允许循迹控制电机。 */
} My_gray_pid_control_t;

extern volatile My_gray_pid_control_t My_gray_pid_control_My;

/**
  * @brief 读取 8 路灰度传感器的原始数字电平。
  * @retval 8 路电平组成的位图；Gray_1 为最高位，Gray_8 为最低位
  * @note GPIO 高电平对应位值 1，低电平对应位值 0。
  */
uint8_t My_gray_read_My(void);

/**
  * @brief 初始化灰度循迹控制器，PID 三个参数均置 0，默认不启动电机。
  */
void My_gray_pid_init_My(void);

/**
  * @brief 设置灰度循迹 PID 参数并清空历史状态。
  * @param kp 比例系数
  * @param ki 积分系数
  * @param kd 微分系数
  */
void My_gray_pid_set_parameters_My(float kp, float ki, float kd);

/**
  * @brief 设置循迹基础速度和最大转向修正量。
  * @param base_pwm 直行基础 PWM，范围 0 到 1000
  * @param correction_limit 修正量绝对值上限，范围 0 到 1000
  */
void My_gray_pid_set_motion_My(int16_t base_pwm, float correction_limit);

/**
  * @brief 设置赛道线对应的数字电平。
  * @param active_level 0 表示低电平检测到线，非零表示高电平检测到线
  */
void My_gray_pid_set_active_level_My(uint8_t active_level);

/**
  * @brief 设置转向修正方向。
  * @param direction 大于等于 0 使用默认方向，小于 0 时反向
  */
void My_gray_pid_set_steering_direction_My(int8_t direction);

/**
  * @brief 启动灰度循迹控制，启动时清空 PID 历史状态。
  */
void My_gray_pid_start_My(void);

/**
  * @brief 停止灰度循迹控制并停止四个电机。
  */
void My_gray_pid_stop_My(void);

/**
  * @brief 执行一次灰度采样、PID 计算和电机输出。
  * @note 应由 10 ms 周期的定时器中断调用；丢线时保持最后一次有效电机输出，重新识别后恢复计算。
  */
void My_gray_pid_update_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_GRAY_H */
