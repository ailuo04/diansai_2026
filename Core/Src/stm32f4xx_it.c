/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /*
   * 不可屏蔽中断通常由时钟安全系统等严重硬件事件触发。当前工程没有恢复路径，
   * 因此不访问外设或共享状态，直接进入下方陷阱循环，保留现场供调试器检查。
   */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   /* 中断上下文永久停留于此；不得加入阻塞通信、动态分配或复杂故障处理。 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /*
   * 硬故障表示处理器无法由其他异常恢复。当前不修改堆栈和全局状态，
   * 保留故障寄存器及调用现场，便于通过调试器定位非法访问或执行错误。
   */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* 故障锁死是安全终态；禁止在未知内存状态下继续控制电机或调用 HAL。 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* 内存管理异常来自 MPU 权限或地址属性违规；不尝试返回，避免重复触发异常。 */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* 保留异常现场供调试器读取 MMFSR、MMFAR 和堆栈帧。 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* 总线故障表示取指或数据访问失败；不再访问可能失效的外设总线和共享资源。 */
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* 保留 BFAR、BFSR 和异常堆栈，防止错误返回后造成不可预测的执行。 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* 用法故障由非法指令、非法状态或受控算术错误触发，当前策略为就地停机。 */
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* 不调用任何依赖正常栈和运行库状态的函数，仅等待调试器接管。 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* 当前裸机工程未使用 SVC 系统调用，不读取堆栈参数，也不修改调度状态。 */
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  /* 空处理后返回被中断代码；若后续引入 RTOS，应由内核接管此异常入口。 */
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* 调试监视异常由断点、观察点等调试事件触发，当前不附加业务处理。 */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* 保持处理时间最短并直接返回，避免调试事件影响实时控制时序。 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* 当前裸机工程未使用 PendSV 做任务切换，不访问主循环或中断共享状态。 */
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  /* 空处理后返回；引入 RTOS 时必须删除冲突实现并交由内核保存、恢复上下文。 */
  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /*
   * SysTick 每 1 ms 触发一次。下方 HAL_IncTick 只递增 HAL 全局时基，
   * 供 HAL_Delay 和超时判断读取；中断内不执行显示、打印或业务轮询。
   */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* HAL 时基已更新，无额外共享状态需要处理，立即退出以限制中断占用时间。 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
  /*
   * DMA1 Stream1 服务 USART3 RX。HAL 将读取并清除 DMA 标志，更新接收状态，
   * 并在满足条件时调用接收事件/错误回调；缓冲区可能与主循环通信队列共享。
   */
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
  /* HAL 已完成标志清除和回调分发，此处不追加耗时处理，避免延迟后续字节接收。 */
  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
  /*
   * DMA1 Stream3 服务 USART3 TX。HAL 将清除传输完成/错误标志并调用发送回调，
   * 发送回调会推进与主循环共享的环形队列，因此其状态更新必须保持原子性。
   */
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */
  /* 回调分发完成后立即退出，不在中断中等待串口物理发送结束或执行 printf。 */
  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  /*
   * USART3 全局中断处理空闲线及 UART 错误等事件。HAL 会读取并清除对应标志，
   * 随后调用 My_uart 中的接收事件或错误回调，异步状态与主循环轮询共享。
   */
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  /* HAL 已完成事件分发；业务解析留在主循环，避免阻塞其他 DMA 和控制中断。 */
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  /*
   * 中断来源为 TIM6 的 10 ms 更新事件；本工程未启用 DAC 欠载业务。HAL 处理函数
   * 将读取并清除 TIM6 更新标志，再调用 main.c 中的 HAL_TIM_PeriodElapsedCallback。
   * 回调会更新 volatile 编码器调试数据、转向位置环、灰度 PID 状态和电机输出；
   * 任务 2 完成反向制动并清零电机输出后，还会冻结与主循环共享的任务计时状态。
   * 计时模块以极短临界区读取多字段一致快照，其他共享多字段状态也不能假设跨字段
   * 读取具有原子性。
   */
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  /*
   * HAL 已按规定顺序完成标志清除和周期回调。此处不追加延时、打印、动态分配、
   * OLED/软件 I2C 等阻塞操作，确保总处理时间显著小于 10 ms 并及时退出中断。
   */
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */


/* USER CODE END 1 */
