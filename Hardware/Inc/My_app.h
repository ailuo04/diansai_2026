#ifndef MY_APP_H
#define MY_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
  * @brief 初始化应用层及其依赖的业务模块。
  * @details 必须在 CubeMX 完成 GPIO、DMA、定时器和串口初始化后调用。
  * @retval HAL_OK 表示关键模块均已启动，HAL_ERROR 表示系统不得进入主循环
  */
HAL_StatusTypeDef My_app_init_My(void);

/**
  * @brief 执行一次主循环应用调度。
  * @details 负责姿态解析、通信轮询、界面刷新和按键任务状态机；函数包含软件
  *          I2C 和 OLED 访问，只能在主循环调用，禁止在中断上下文执行。
  */
void My_app_process_My(void);

/**
  * @brief 执行一次 10 ms 周期控制任务。
  * @details 仅由 TIM6 周期中断回调调用，内部不得加入阻塞、打印或动态内存操作。
  */
void My_app_control_10ms_My(void);

#ifdef __cplusplus
}
#endif

#endif
