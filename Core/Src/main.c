/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HWT101.h"
#include "My_move.h"
#include "My_uart.h"
#include "My_oled.h"
#include "My_gray.h"
#include "My_key.h"
#include "My_timer.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MY_TASK_GRAY_TRACKING 2U /* 任务2固定执行灰度循迹，其他任务不得启动该控制器。 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern volatile float Angle;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /*
   * 主循环状态只在 main 上下文中读写，不与中断直接共享；中断侧数据由各驱动
   * 的接口读取，避免主循环直接访问异步更新的内部缓冲区。
   */
  uint32_t elapsed_ms; /* 定时器模块返回的累计毫秒数，用于换算并刷新 OLED 计时行。 */
  uint8_t gray_value; /* 8 路灰度输入快照，位 7~0 依次对应从左到右的传感器。 */
  uint8_t gray_index; /* 位图转显示字符串时的传感器索引，仅在当前循环迭代内使用。 */
  char gray_text[9] = "00000000"; /* 8 个状态字符加结尾空字符，供 OLED 按字符串输出。 */
  uint8_t key_value = MY_KEY_VALUE_MIN; /* 待确认任务编号，短按时在合法范围内循环选择。 */
  uint8_t key_value_confirmed = 0U; /* 长按确认锁存标志；置位后短按不再改变任务编号。 */
  My_key_event_t key_event; /* 按键消抖模块在本轮扫描中产生的离散事件。 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /*
   * HAL_Init 已完成外设复位、Flash 接口和 1 ms SysTick 基础初始化。
   * 此处不启动业务模块，所有依赖系统时钟的外设必须等待时钟配置完成。
   */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*
   * 系统时钟已切换到 HSE+PLL；随后由 CubeMX 依次初始化 GPIO、DMA、TIM5、
   * USART3 和 TIM6。DMA 必须先于 USART3 业务接收启动，TIM6 则在巡线参数
   * 装载完成后才允许启动中断，避免控制回调读取未初始化状态。
   */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /*
   * 先建立 USART3 DMA 收发状态机。该接口失败时通信缓冲区不可用，立即进入
   * Error_Handler，防止后续 printf 或姿态数据接收访问未初始化资源。
   */
  if (My_uart_init_My() != HAL_OK)
  {
    Error_Handler();
  }
  /* 清空 HWT101 姿态解析状态，确保首次显示不会沿用无效的历史帧。 */
  HWT101_getAngle_Reset();
  /*
   * OLED 初始化成功后把逻辑光标归零；失败只通过串口报告并继续运行，
   * 因为显示器不是运动和通信模块的启动前置条件。
   */
  if (My_oled_init_My() == HAL_OK)
  {
    My_oled_set_cursor_My(0U, 0U);
  }
  else
  {
    printf("OLED NACK: check PF3/PF4, power and address\r\n");
  }
  /*
   * 初始化电机 PWM、方向控制和编码器反馈。失败时禁止继续进入控制循环，
   * 避免在执行器状态未知的情况下输出运动指令。
   */
  if (My_move_init_My() != HAL_OK)
  {
    Error_Handler();
  }
  /*
   * 建立灰度循迹 PID 状态并装载低速基线参数。位置环按 TIM6 的 10 ms 周期
   * 更新；积分项暂时关闭，避免尚未完成实车标定时产生积分累积和突发输出。
   */
  My_gray_pid_init_My();
  My_gray_pid_set_parameters_My(25.0f, 0.0f, 8.0f);
  My_gray_pid_set_motion_My(250, 200.0f);
  /* 实测黑线对应高电平，按高电平有效计算位置误差，避免把白色区域误判为赛道线。 */
  My_gray_pid_set_active_level_My(1U);
  My_gray_pid_set_steering_direction_My(1);
  /* 初始化按键消抖和长按计时状态，使首次扫描从稳定释放状态开始。 */
  My_key_init_My();
  /*
   * 启动 10 ms 控制定时器。此时巡线使能仍为 0，中断只更新传感器快照，
   * 不会驱动电机；若定时器无法启动，则进入安全停机路径，禁止无周期控制。
   */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* 串口提示仅表示软件初始化结束；实际 DMA 发送由驱动异步完成。 */
  printf("USART3 DMA ready\r\n");
	/* 启动任务计时基准；若计时应从赛题动作开始，需要把本调用移到对应状态转换处。 */
	My_timer_start_My();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*
		 * 请求 HWT101 驱动解析当前可用姿态数据，再轮询 USART3 软件队列。
		 * 两个接口均在主循环处理业务，避免把格式解析和回调业务放入中断上下文。
		 */
		HWT101_getAngle();
    My_uart_poll_My();
		/* 显示最新角度快照；Angle 由姿态解析模块更新，显示精度固定为 0.001 度。 */
		My_oled_printf_at_My(1U, 1U, "角度：%-7.3f°",Angle);
		/*
		 * 读取单调递增的毫秒计时值，并拆分为整秒和十分之一秒。
		 * OLED 驱动会抑制未变化区域，但该调用仍属于轮询刷新，不得移入中断。
		 */
		elapsed_ms = My_timer_get_elapsed_ms_My();
		My_oled_printf_at_My(2U, 1U, "计时:%5u.%us",
			(unsigned int)(elapsed_ms / 1000U),
			(unsigned int)((elapsed_ms / 100U) % 10U));
    /*
     * 一次性采集 8 路灰度输入，保证同一显示帧使用一致快照；随后按掩码从左到右
     * 转换为字符。gray_text[8] 保持初始化时的空字符，不在循环中覆盖。
     */
    gray_value = My_gray_read_My();
    for (gray_index = 0U; gray_index < 8U; gray_index++)
    {
      gray_text[gray_index] =
        (gray_value & (uint8_t)(MY_GRAY_1_MASK >> gray_index)) != 0U ? '1' : '0';
    }
    My_oled_printf_at_My(3U, 1U, "灰度：%s", gray_text);

    /*
     * 扫描按键消抖状态机：未确认时短按令任务号在最小值和最大值之间循环；
     * 长按将选择永久锁存。当前逻辑没有取消确认路径，重新选择需复位或扩展状态机。
     */
    key_event = My_key_scan_My();
    if (key_event == MY_KEY_EVENT_SHORT_PRESS && key_value_confirmed == 0U)
    {
      key_value++;
      if (key_value > MY_KEY_VALUE_MAX)
      {
        key_value = MY_KEY_VALUE_MIN;
      }
    }
    else if (key_event == MY_KEY_EVENT_LONG_PRESS && key_value_confirmed == 0U)
    {
      /*
       * 首次长按释放后锁存当前任务号。只有任务2允许启动灰度循迹；启动接口会
       * 清空 PID 历史状态，随后由 TIM6 中断根据灰度位置更新四路电机输出。
       * 其他任务当前尚未实现运动逻辑，必须显式停止循迹和电机，避免控制器残留
       * 状态导致误动作。确认后任务号不可再次修改，重新选择需要复位系统。
       */
      key_value_confirmed = 1U;
      if (key_value == MY_TASK_GRAY_TRACKING)
      {
        My_gray_pid_start_My();
      }
      else
      {
        My_gray_pid_stop_My();
      }
    }
    /* 将任务号及锁存状态写入 OLED，便于操作者在启动动作前核对选择结果。 */
    My_oled_printf_at_My(4U, 1U, key_value_confirmed != 0U ?
      "任务:%u 已确认" : "任务:%u 未确认", (unsigned int)key_value);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief HAL 定时器周期中断统一回调。
  * @details HAL 在定时器更新中断清除硬件标志后调用本函数。本项目只响应 TIM6，
  *          用它驱动灰度循迹 PID 的固定周期更新；其他定时器回调直接返回。
  *          该函数运行于中断上下文，会读写 PID 和电机控制共享状态，相关变量
  *          必须保持 volatile/原子访问约束。回调内禁止阻塞、延时、打印和动态分配。
  * @param htim 触发中断的定时器句柄
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 过滤非 TIM6 中断，避免其他 HAL 定时器意外触发运动控制更新。 */
  if (htim->Instance == TIM6)
  {
    /* 执行单次定周期 PID 计算并更新电机输出；计算耗时必须小于 TIM6 周期。 */
    My_gray_pid_update_My();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
