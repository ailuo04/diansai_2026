#ifndef MY_BALL_BALANCE_H
#define MY_BALL_BALANCE_H /* 防止钢球视觉外环接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "My_pid.h"

#define MY_BALL_FRAME_LENGTH 13U /* 摄像头固定帧长度：帧头2字节、位置4字节、速度5字节、帧尾2字节。 */
#define MY_BALL_POSITIVE_TARGET_MM  50 /* 第一段目标：相机坐标 +50 mm。 */
#define MY_BALL_NEGATIVE_TARGET_MM (-50) /* 第二段目标：相机坐标 -50 mm。 */
#define MY_BALL_POSITIVE_SWITCH_MIN_MM 40 /* 任务三第一段进入该位置后立即切换到 -50 mm。 */

#define MY_BALL_ZERO_TARGET_MM                    0
#define MY_BALL_ZERO_CONTROL_PERIOD_S             (0.010f) /* TIM6 固定控制周期，修改定时器后必须同步更新。 */

/*
 * 任务 3 专用位置式 PID 参数。P/I 项使用摄像头返回的位置误差，D 项直接使用同一帧
 * 返回的钢球速度；任务切换时会重新装载这些参数，避免被任务 4/5 的 0 点保持参数覆盖。
 */
#define MY_BALL_TASK3_PID_KP_DEG_PER_MM           (0.45f)  /* 任务 3 两段目标追踪的位置误差比例增益。 */
#define MY_BALL_TASK3_PID_KI_DEG_PER_MM_S         (0.5f)   /* 任务 3 两段目标追踪的位置误差时间积分增益。 */
#define MY_BALL_TASK3_PID_KD_DEG_PER_MM_S         (0.50f)  /* 任务 3 两段目标追踪的钢球速度微分反馈增益。 */
#define MY_BALL_TASK3_INTEGRAL_LIMIT_MM_S         (1000.0f) /* 任务 3 位置误差时间积分的绝对限幅。 */
#define MY_BALL_TASK3_INTEGRAL_DISTANCE_MM        (100.0f) /* 任务 3 允许积分的位置误差范围。 */
#define MY_BALL_TASK3_INTEGRAL_SPEED_MM_S         (60.0f)  /* 任务 3 允许积分的钢球速度范围。 */
#define MY_BALL_TASK3_INTEGRAL_DECAY              (0.98f)  /* 任务 3 积分条件不满足时的逐周期释放比例。 */
#define MY_BALL_TASK3_CORRECTION_LIMIT_DEG        (45.0f)  /* 任务 3 电机输出轴相对水平角的最大连续修正量。 */
#define MY_BALL_TASK3_CORRECTION_STEP_DEG         (2.5f)   /* 任务 3 每 10 ms 允许变化的最大修正角。 */

/*
 * 任务 4/5 专用 0 点保持 PID 参数。任务 3 与任务 4/5 使用不同 PID 实例和修正角历史，
 * 因此两类任务的参数、积分、微分与变化率限制状态互不影响。
 */
#define MY_BALL_ZERO_PID_KP_DEG_PER_MM             (0.45f)  /* 位置误差比例增益，数值保持与原串级控制的小误差区一致。 */
#define MY_BALL_ZERO_PID_KI_DEG_PER_MM_S           (0.5f) /* 位置误差时间积分增益，用于补偿平台水平零偏。 */
#define MY_BALL_ZERO_PID_KD_DEG_PER_MM_S           (0.50f)  /* 钢球速度微分反馈增益；输出中按速度反方向施加。 */
#define MY_BALL_ZERO_INTEGRAL_LIMIT_MM_S          (1000.0f) /* 位置误差时间积分的绝对限幅。 */
#define MY_BALL_ZERO_INTEGRAL_DISTANCE_MM         (100.0f) /* 位置误差不超过 100 mm 时允许积分，仍受速度和积分限幅保护。 */
#define MY_BALL_ZERO_INTEGRAL_SPEED_MM_S          (60.0f)  /* 只在低速时允许积分，避免运动中反冲。 */
#define MY_BALL_ZERO_INTEGRAL_DECAY               (0.98f)  /* 积分条件不满足时逐周期释放历史偏置。 */
#define MY_BALL_ZERO_CORRECTION_LIMIT_DEG         (45.0f)  /* 电机输出轴相对水平角的最大连续修正量。 */
#define MY_BALL_ZERO_CORRECTION_STEP_DEG          (2.5f)   /* 每 10 ms 允许变化的最大修正角，抑制冲击。 */
#define MY_BALL_ZERO_ARRIVE_DISTANCE_MM           (10.0f)   /* 进入中心保持状态的位置误差阈值。 */
#define MY_BALL_ZERO_ARRIVE_SPEED_MM_S            (15.0f)  /* 进入中心保持状态的估计速度阈值。 */

/*
 * ==================== 钢球控制调参区 ====================
 * 任务 3 的 +50 mm、-50 mm 两段使用 MY_BALL_TASK3_* 参数；任务 4/5 的 0 mm 保持
 * 使用 MY_BALL_ZERO_* 参数。两类任务各自保留 PID 和角度变化率历史，任务切换时只
 * 复位对应实例，避免积分、微分和现场调参互相串扰。
 */
#define MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG (-95.0f) /* 平台实测水平角。 */
#define MY_BALL_TERMINAL_MOTOR_ANGLE_DEG   (-114.0f) /* 终止时电机输出轴的绝对目标角度，单位度。 */
#define MY_BALL_TERMINAL_ANGLE_TOLERANCE_DEG (0.3f) /* 实际角度进入该范围后确认终止角度到位并关闭驱动。 */
#define MY_BALL_CONTROL_DIRECTION           (-1.0f)  /* 目标方向到电机角方向的映射；实测原方向相反，因此整体翻转驱动和制动修正。 */

/* 到点判断一般不需要现场调整：必须同时满足距离和速度条件，防止高速掠过时误切换。 */
#define MY_BALL_ARRIVE_DISTANCE_MM         (5.0f)   /* 距离目标不超过该值才判断到点。 */
#define MY_BALL_ARRIVE_SPEED_MM_S          (40.0f)  /* 到点时速度还必须低于该值。 */
#define MY_BALL_FINAL_ARRIVE_CONFIRM_FRAMES 3U      /* 最终到点至少需要连续满足条件的新摄像头帧数。 */
#define MY_BALL_FINAL_ARRIVE_CONFIRM_MS    10U     /* 最终到点条件至少持续该时间才锁存，单位毫秒。 */

/* 下列参数只用于通信保护和输入约束，不参与现场控制手感调节。 */
#define MY_BALL_SPEED_ABS_LIMIT_MM_S       (2000.0f) /* 摄像头协议速度参与控制前的绝对限幅。 */
#define MY_BALL_COMMUNICATION_TIMEOUT_MS   200U     /* 超时后停止使用旧数据并让平台回水平。 */

#define MY_BALL_CONTROL_STATE_HOLD  0U /* 正在确认到点、最终到点或无有效数据，平台保持水平。 */
#define MY_BALL_CONTROL_STATE_DRIVE 1U /* PID 输出正在推动钢球接近当前目标。 */
#define MY_BALL_CONTROL_STATE_BRAKE 2U /* PID 输出方向与钢球速度相反，正在执行制动。 */

/**
  * @brief 钢球摄像头协议和单轴平衡外环运行状态。
  * @details 原始位置、协议速度和接收统计由主循环更新；新数据提交时短暂屏蔽中断，
  *          保证 TIM6 读取到同一帧快照。限幅后的摄像头速度、PID 分项、目标角和
  *          新鲜度由 TIM6 更新。本结构用于 Keil Watch 观察，不应由其他模块直接改写。
  */
typedef struct
{
  int16_t position_mm;             /**< 最近合法帧的钢球位置，左负右正，单位毫米。 */
  int16_t speed_mm_s;              /**< 摄像头帧携带的原始速度，是钢球 PID 的速度反馈来源。 */
  int16_t target_position_mm;      /**< 当前钢球目标；任务 3 使用分段目标，任务 4/5 使用 0 mm。 */
  float position_speed_mm_s;       /**< 摄像头返回并完成物理限幅的钢球速度，单位毫米每秒。 */
  float measured_speed_mm_s;       /**< 本周期实际参与控制的钢球速度，单位毫米每秒。 */
  float pid_p_angle_deg;           /**< 钢球 PID 比例项经机构方向映射后的角度贡献。 */
  float pid_i_angle_deg;           /**< 钢球 PID 积分项经机构方向映射后的角度贡献。 */
  float pid_d_angle_deg;           /**< 摄像头速度微分项经机构方向映射后的角度贡献。 */
  float pid_output_angle_deg;      /**< 钢球 PID 限幅后、变化率限制前的目标修正角。 */
  float distance_to_target_mm;     /**< 当前目标与测量位置之间的距离绝对值，单位毫米。 */
  float correction_angle_deg;      /**< 当前驱动、制动或保持动作对应的平台修正角。 */
  float target_motor_angle_deg;    /**< 下发给电机位置内环的目标输出轴角度，单位度。 */
  uint32_t last_update_ms;         /**< 最近合法帧到达时的 HAL 毫秒时刻。 */
  uint32_t final_arrival_start_ms; /**< 最终到点连续确认窗口的首帧时刻，未确认时为零。 */
  uint32_t valid_frame_count;      /**< 已通过格式和范围校验的帧数量。 */
  uint32_t invalid_frame_count;    /**< 帧头后续格式、帧尾或位置范围非法的帧数量。 */
  uint8_t parser_index;            /**< 当前已缓存的候选帧字节数，范围 0～12。 */
  uint8_t has_measurement;         /**< 非零表示至少接收过一帧合法数据。 */
  uint8_t data_fresh;              /**< 非零表示最近合法帧未超过通信超时。 */
  uint8_t position_speed_ready;    /**< 非零表示当前控制周期使用了有效摄像头速度。 */
  uint8_t control_state;           /**< 当前控制动作：保持、驱动或制动。 */
  uint8_t positive_target_reached; /**< 非零表示位置进入 40～50 mm 区间，当前目标已切换到 -50 mm。 */
  uint8_t final_arrival_settling;  /**< 非零表示 -50 mm 已进入带滞回的最终稳定确认阶段。 */
  uint8_t final_arrival_confirm_count; /**< 已连续满足最终到点条件的新摄像头帧数量。 */
  uint8_t negative_target_reached; /**< 非零表示 -50 mm 已稳定到点，控制器已锁存最终保持。 */
  uint8_t terminal_angle_commanded; /**< 非零表示 -50 mm 已确认到点并锁存终止角度目标。 */
  uint8_t terminal_angle_reached;   /**< 非零表示电机已到达终止角度允许误差范围并进入断电保持。 */
} My_ball_balance_debug_t;

extern volatile My_ball_balance_debug_t My_ball_balance_debug_My;

/**
  * @brief 任务 3 分段钢球位置外环专用 PID 实例。
  * @details 只由任务 3 的 TIM6 控制路径更新；任务 3 启动时装载 MY_BALL_TASK3_* 参数。
  */
extern volatile My_pid_t My_ball_task3_pid_My;

/**
  * @brief 任务 4/5 钢球 0 点保持外环专用 PID 实例。
  * @details 只由任务 4/5 的 TIM6 控制路径更新；任务 4/5 启动时装载 MY_BALL_ZERO_* 参数。
  */
extern volatile My_pid_t My_ball_zero_pid_My;

/**
  * @brief 初始化钢球摄像头流式解析器和单轴平衡外环。
  * @details 在主循环和 TIM6 中断启动前调用，位置目标从相机坐标 +50 mm 开始；
  *          电机目标角在收到首帧前保持当前平台水平基准角。
  */
void My_ball_balance_init_My(void);

/**
  * @brief 复位钢球控制目标、测速状态和当前动作。
  * @details 任务 3 每次被确认前调用，使控制器从 +50 mm 第一阶段重新开始；
  *          函数不访问阻塞外设，调用方若与 TIM6 并发，应在外部短暂屏蔽中断。
  */
void My_ball_balance_reset_control_My(void);

/**
  * @brief 复位任务 4/5 共用的钢球 0 点控制状态。
  * @details 保留最近一次摄像头测量，但把目标设为 0 mm，并清除任务 3 的分段、终止
  *          和到点锁存标志；调用方若与 TIM6 并发，必须在外部短暂屏蔽中断。
  */
void My_ball_balance_reset_zero_control_My(void);

/**
  * @brief 向钢球协议解析器输入一段 USART3 字节流。
  * @param data 本次收到的数据首地址
  * @param length 本次收到的字节数
  * @details 本函数允许一段数据包含半帧、一帧或多帧；解析器按 `ly` 帧头和固定
  *          13 字节长度重组，并校验符号、数字、`cs` 帧尾及 ±125 mm 位置范围。
  *          本函数只在主循环调用，不在串口中断内进行格式解析。
  */
void My_ball_balance_feed_My(const uint8_t *data, uint16_t length);

/**
  * @brief 执行一次任务 3 分段钢球位置式 PID 并更新电机位置目标。
  * @details 仅由 TIM6 周期中断调用，执行定长运算，不阻塞、不打印、不分配内存。
  *          钢球位置进入 40～50 mm 区间后立即切换到 -50 mm；P/I 项使用当前位置误差，
  *          D 项直接使用摄像头返回速度，通信超时后清除 PID 并平滑回到水平。
  */
void My_ball_balance_update_10ms_My(void);

/**
  * @brief 执行一次任务 4/5 共用的钢球 0 点位置式 PID 控制。
  * @details 仅由 TIM6 周期中断调用，任务 4/5 行驶和停车阶段都会执行。PID 的 P/I
  *          项使用摄像头位置误差，D 项直接使用同一帧返回的反向钢球速度反馈。
  *          通信超时后清除 PID 历史并按角度变化率平滑回到水平。
  */
void My_ball_balance_update_zero_10ms_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_BALL_BALANCE_H */
