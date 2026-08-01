#ifndef MY_GRAY_H
#define MY_GRAY_H /* 防止灰度循迹接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "My_pid.h"

#define MY_GRAY_1_MASK (0x80U) /* 灰度位图中第 1 路传感器对应的位掩码。 */
#define MY_GRAY_2_MASK (0x40U) /* 灰度位图中第 2 路传感器对应的位掩码。 */
#define MY_GRAY_3_MASK (0x20U) /* 灰度位图中第 3 路传感器对应的位掩码。 */
#define MY_GRAY_4_MASK (0x10U) /* 灰度位图中第 4 路传感器对应的位掩码。 */
#define MY_GRAY_5_MASK (0x08U) /* 灰度位图中第 5 路传感器对应的位掩码。 */
#define MY_GRAY_6_MASK (0x04U) /* 灰度位图中第 6 路传感器对应的位掩码。 */
#define MY_GRAY_7_MASK (0x02U) /* 灰度位图中第 7 路传感器对应的位掩码。 */
#define MY_GRAY_8_MASK (0x01U) /* 灰度位图中第 8 路传感器对应的位掩码。 */
#define MY_GRAY_TRACK_MASK ((uint8_t)(MY_GRAY_4_MASK | MY_GRAY_5_MASK | MY_GRAY_6_MASK | MY_GRAY_7_MASK | MY_GRAY_8_MASK)) /* 实际参与循迹加权计算的 Gray_4～Gray_8 通道集合。 */
#define MY_GRAY_STOP_LEFT_PATTERN ((uint8_t)(MY_GRAY_4_MASK | MY_GRAY_5_MASK | MY_GRAY_6_MASK)) /* 停车标志左侧候选三连黑组合。 */
#define MY_GRAY_STOP_MIDDLE_PATTERN ((uint8_t)(MY_GRAY_5_MASK | MY_GRAY_6_MASK | MY_GRAY_7_MASK)) /* 停车标志中间候选三连黑组合。 */
#define MY_GRAY_STOP_RIGHT_PATTERN ((uint8_t)(MY_GRAY_6_MASK | MY_GRAY_7_MASK | MY_GRAY_8_MASK)) /* 停车标志右侧候选三连黑组合。 */
#define MY_GRAY_STOP_CONFIRM_CYCLES 5U /* 停车标志必须连续满足的 10 毫秒采样次数，用于过滤瞬时误判。 */

#define MY_GRAY_PID_DEFAULT_BASE_PWM         250    /* 灰度控制器初始化时使用的默认直行基础 PWM。 */
#define MY_GRAY_PID_DEFAULT_CORRECTION_LIMIT 300.0f /* 灰度位置 PID 默认转向修正绝对值上限。 */
#define MY_GRAY_PID_DEFAULT_ACTIVE_LEVEL     0U     /* 默认以低电平表示检测到黑线。 */
#define MY_GRAY_RAMP_TIME_MS                 3000U  /* 任务启动时 S 型速度曲线的持续时间，单位毫秒。 */
#define MY_GRAY_CONTROL_PERIOD_MS            10U    /* TIM6 灰度控制更新周期，单位毫秒。 */
#define MY_GRAY_TASK2_SLOWDOWN_TIME_MS      13000U /* 任务 2 从启动计时起进入低速段的时刻，单位毫秒。 */
#define MY_GRAY_TASK2_SLOW_BASE_PWM           250  /* 13 秒后使用的低速段基础 PWM；增大可提速，但会缩短停车标志识别时间。 */
#define MY_GRAY_REVERSE_BRAKE_PWM            120 /* 识别停车标志后施加的反向开环制动 PWM。 */
#define MY_GRAY_REVERSE_BRAKE_TIME_MS        50U /* 反向制动持续时间，单位毫秒。 */

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
  uint8_t stop_confirm_count; /* 右侧五路中相邻三连黑的连续确认次数，用于过滤瞬时误判。 */
  uint8_t reverse_brake_cycles; /* 停止线触发后剩余的反向制动周期数，仅由 10 ms 控制中断更新。 */
  uint16_t ramp_step;     /* 当前 S 曲线步号，范围 0 到 MY_GRAY_RAMP_STEPS。 */
  float ramp_start_pwm;   /* 本次启动加速曲线的起始基础 PWM。 */
  float ramp_pwm;         /* 当前经过 S 曲线平滑后的基础 PWM。 */
} My_gray_pid_control_t;

extern volatile My_gray_pid_control_t My_gray_pid_control_My; /* TIM6 中断读写的灰度循迹控制器状态。 */

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
  * @brief 启动灰度循迹控制，启动时清空 PID 历史状态并执行 S 型加速。
  */
void My_gray_pid_start_My(void);

/**
  * @brief 立即停止灰度循迹并同步清零四轮电机输出，不执行 S 型减速。
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
