#ifndef MY_APP_H
#define MY_APP_H /* 防止应用层接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/*
 * 影子调试模式默认开启：底盘四轮始终保持零 PWM，但平台倾角电机、摄像头解析、
 * 钢球观测器和控制闭环均正常运行。可手动推动小车或扰动钢球模拟加速度，同时
 * 观察倾角电机的实际补偿效果；改为 0U 并重新编译后恢复任务 4/5 底盘运动。
 */
#define MY_APP_SHADOW_DEBUG_MODE 0U

/**
  * @brief 初始化应用层及其依赖的业务模块。
  * @details 必须在 CubeMX 完成 GPIO、DMA、定时器和串口初始化后调用。
  * @retval HAL_OK 表示关键模块均已启动，HAL_ERROR 表示系统不得进入主循环
  */
HAL_StatusTypeDef My_app_init_My(void);

/**
  * @brief 执行一次主循环应用调度。
  * @details 负责姿态读取、摄像头字节流解析、界面刷新和按键任务状态机；函数包含
  *          软件 I2C 和 OLED 访问，只能在主循环调用，禁止在中断上下文执行。
  */
void My_app_process_My(void);

/**
  * @brief 执行一次 10 ms 周期控制任务。
  * @details 仅由 TIM6 周期中断回调调用，先更新转向编码器与四轮周期速度，再更新
  *          任务 3 钢球外环、任务 4 的定时底盘曲线、任务 5 的稳定循迹或任务 6 的
  *          启动目标循迹；任务 4/5 保持 0 点，任务 6 保持启动采样目标，最后更新
  *          转向位置内环和任务 2/5/6 灰度循迹。
  *          内部不得加入阻塞、打印、动态内存操作或不定长循环。
  */
void My_app_control_10ms_My(void);

#ifdef __cplusplus
}
#endif

#endif
