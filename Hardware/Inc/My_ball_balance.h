#ifndef MY_BALL_BALANCE_H
#define MY_BALL_BALANCE_H /* 防止钢球视觉外环接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MY_BALL_FRAME_LENGTH 13U /* 摄像头固定帧长度：帧头2字节、位置4字节、速度5字节、帧尾2字节。 */
#define MY_BALL_POSITIVE_TARGET_MM  45 /* 第一段目标：相机坐标 +50 mm。 */
#define MY_BALL_NEGATIVE_TARGET_MM (-40) /* 第二段目标：相机坐标 -50 mm。 */
#define MY_BALL_TERMINAL_POSITION_MM (-25) /* 终止触发位置：钢球到达 -40 mm 后结束外环。 */

/*
 * ==================== 钢球控制调参区 ====================
 * 控制只有三种动作：向目标小角度驱动、反向制动、到点后回水平。
 * 平台标定只涉及水平角和控制方向；驱动角、制动角、最高速度与制动距离均按
 * 目标阶段独立配置，便于分别整定 0 到 +50 mm 和 +50 到 -50 mm 两段行程。
 */
#define MY_BALL_HORIZONTAL_MOTOR_ANGLE_DEG (-95.0f) /* 平台实测水平角。 */
#define MY_BALL_TERMINAL_MOTOR_ANGLE_DEG   (-114.0f) /* 终止时电机输出轴的绝对目标角度，单位度。 */
#define MY_BALL_TERMINAL_ANGLE_TOLERANCE_DEG (0.3f) /* 实际角度进入该范围后确认终止角度到位并关闭驱动。 */
#define MY_BALL_CONTROL_DIRECTION          (-1.0f)  /* 目标方向到电机角方向的映射，跑反时改符号。 */
#define MY_BALL_POSITIVE_DRIVE_ANGLE_DEG   (30.0f) /* 0 到 +50 mm 第一段使用的驱动修正角。 */
#define MY_BALL_POSITIVE_BRAKE_ANGLE_DEG   (40.0f) /* 第一段触发制动后的反向平台修正角。 */
#define MY_BALL_POSITIVE_MAX_SPEED_MM_S    (40.0f) /* 0 到 +50 mm 第一段的制动速度阈值。 */
#define MY_BALL_POSITIVE_BRAKE_DISTANCE_MM (20.0f) /* 第一段距 +50 mm 目标进入该范围后制动。 */
#define MY_BALL_NEGATIVE_DRIVE_ANGLE_DEG   (30.0f) /* +50 到 -50 mm 第二段使用的驱动修正角。 */
#define MY_BALL_NEGATIVE_BRAKE_ANGLE_DEG   (20.0f) /* 第二段使用更强制动；再增大会触及 -100° 软限位。 */
#define MY_BALL_NEGATIVE_MAX_SPEED_MM_S    (0.0f) /* +50 到 -50 mm 第二段使用较低阈值提前控速。 */
#define MY_BALL_NEGATIVE_BRAKE_DISTANCE_MM (0.0f) /* 第二段距 -50 mm 目标 40 mm 时开始提前制动。 */

/* 到点判断一般不需要现场调整：必须同时满足距离和速度条件，防止高速掠过时误切换。 */
#define MY_BALL_ARRIVE_DISTANCE_MM         (5.0f)   /* 距离目标不超过该值才判断到点。 */
#define MY_BALL_ARRIVE_SPEED_MM_S          (40.0f)  /* 到点时速度还必须低于该值。 */
#define MY_BALL_FINAL_ARRIVE_CONFIRM_FRAMES 3U      /* 最终到点至少需要连续满足条件的新摄像头帧数。 */
#define MY_BALL_FINAL_ARRIVE_CONFIRM_MS    10U     /* 最终到点条件至少持续该时间才锁存，单位毫秒。 */
#define MY_BALL_FINAL_HOLD_RELEASE_DISTANCE_MM (15.0f) /* 最终确认期间偏离目标超过该距离才恢复控制。 */
#define MY_BALL_FINAL_LEVEL_STEP_DEG       (1.0f)   /* 最终到点窗口内每 10 ms 回水平的最大角度步长。 */

/* 下列参数只用于通信保护和测速，不参与现场控制手感调节。 */
#define MY_BALL_SPEED_ABS_LIMIT_MM_S       (2000.0f) /* 位置差分或协议速度参与控制前的绝对限幅。 */
#define MY_BALL_COMMUNICATION_TIMEOUT_MS   200U     /* 超时后停止使用旧数据并让平台回水平。 */

#define MY_BALL_CONTROL_STATE_HOLD  0U /* 正在确认到点、最终到点或无有效数据，平台保持水平。 */
#define MY_BALL_CONTROL_STATE_DRIVE 1U /* 用固定小角度向当前目标驱动。 */
#define MY_BALL_CONTROL_STATE_BRAKE 2U /* 速度过大、距离过近或运动方向错误时反向制动。 */

/**
  * @brief 钢球摄像头协议和单轴平衡外环运行状态。
  * @details 原始位置、协议速度和接收统计由主循环更新；新数据提交时短暂屏蔽中断，
  *          保证 TIM6 读取到同一帧快照。位置差分速度、控制动作、目标角和新鲜度
  *          由 TIM6 更新。本结构用于 Keil Watch 观察，不应由其他模块直接改写。
  */
typedef struct
{
  int16_t position_mm;             /**< 最近合法帧的钢球位置，左负右正，单位毫米。 */
  int16_t speed_mm_s;              /**< 摄像头帧携带的原始速度，仅用于对照诊断，单位毫米每秒。 */
  int16_t target_position_mm;      /**< 任务 3 当前位置目标，先为 +50 mm，切换后保持 -50 mm。 */
  float position_speed_mm_s;       /**< 由相邻位置帧和时间戳计算的钢球速度，单位毫米每秒。 */
  float measured_speed_mm_s;       /**< 本周期用于制动判断的钢球速度，单位毫米每秒。 */
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
  uint8_t position_speed_ready;    /**< 非零表示位置差分速度已经建立有效时间窗口。 */
  uint8_t control_state;           /**< 当前控制动作：保持、驱动或制动。 */
  uint8_t positive_target_reached; /**< 非零表示 +50 mm 已低速到点，当前目标已切换到 -50 mm。 */
  uint8_t final_arrival_settling;  /**< 非零表示 -50 mm 已进入带滞回的最终稳定确认阶段。 */
  uint8_t final_arrival_confirm_count; /**< 已连续满足最终到点条件的新摄像头帧数量。 */
  uint8_t negative_target_reached; /**< 非零表示 -50 mm 已稳定到点，控制器已锁存最终保持。 */
  uint8_t terminal_angle_commanded; /**< 非零表示已在 -40 mm 触发并锁存终止角度目标。 */
  uint8_t terminal_angle_reached;   /**< 非零表示电机已到达终止角度允许误差范围并进入断电保持。 */
} My_ball_balance_debug_t;

extern volatile My_ball_balance_debug_t My_ball_balance_debug_My;

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
  * @brief 向钢球协议解析器输入一段 USART3 字节流。
  * @param data 本次收到的数据首地址
  * @param length 本次收到的字节数
  * @details 本函数允许一段数据包含半帧、一帧或多帧；解析器按 `ly` 帧头和固定
  *          13 字节长度重组，并校验符号、数字、`cs` 帧尾及 ±125 mm 位置范围。
  *          本函数只在主循环调用，不在串口中断内进行格式解析。
  */
void My_ball_balance_feed_My(const uint8_t *data, uint16_t length);

/**
  * @brief 执行一次 10 ms 钢球固定驱动/制动控制并更新电机位置目标。
  * @details 仅由 TIM6 周期中断调用，执行定长运算，不阻塞、不打印、不分配内存。
  *          钢球先低速到达 +50 mm，再切换到 -50 mm；运动时使用固定小驱动角，
  *          速度过大、距离过近或方向错误时立即反向制动，通信超时后回到水平。
  */
void My_ball_balance_update_10ms_My(void);

#ifdef __cplusplus
}
#endif

#endif /* MY_BALL_BALANCE_H */
