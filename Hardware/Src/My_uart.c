#include "My_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <rt_misc.h>

__asm(".global __use_no_semihosting\n");

/* ARM 标准库需要由应用提供文件流对象，避免启动阶段进入半主机断点。 */
FILE __stdout;
FILE __stdin;

/* DMA 正在写入的原始接收缓冲区。 */
static uint8_t My_uart_rx_dma_buffer[MY_UART_RX_BUFFER_SIZE];
/* 由空闲中断确认后的“四槽环形帧队列”。 */
static uint8_t My_uart_rx_frame_queue[MY_UART_RX_FRAME_QUEUE_SIZE][MY_UART_RX_BUFFER_SIZE];
/* 每个槽位中当前帧的有效字节数。 */
static volatile uint16_t My_uart_rx_frame_lengths[MY_UART_RX_FRAME_QUEUE_SIZE];
/* 指向当前最旧未读帧的槽位索引。 */
static volatile uint8_t My_uart_rx_queue_head;
/* 当前队列中待处理帧的数量，范围为 0~4。 */
static volatile uint8_t My_uart_rx_queue_count;
/* 主循环读取最旧一帧时使用的临时拷贝缓冲区。 */
static uint8_t My_uart_poll_buffer[MY_UART_RX_BUFFER_SIZE];
/* 接收过程中是否发生过队列覆盖、缓冲区截断或恢复异常。 */
static volatile uint8_t My_uart_rx_overflow;
/* 重新启动 DMA 接收流程时是否发生过失败。 */
static volatile uint8_t My_uart_rx_restart_error;

/**
  * @brief 计算环形队列中的下一个槽位索引。
  * @param index 当前槽位索引
  * @retval 下一个槽位索引；到达队尾后会自动回绕到 `0`
  */
static uint8_t My_uart_next_queue_index_My(uint8_t index)
{
  index++;
  if (index >= MY_UART_RX_FRAME_QUEUE_SIZE)
  {
    index = 0U;
  }

  return index;
}

/**
  * @brief 根据进入临界区前的状态恢复中断。
  * @param primask 进入临界区前保存的 `PRIMASK` 值；
  *                当该值为 `0` 时表示进入临界区前中断是开启状态，
  *                此时函数会在退出时重新打开中断
  */
static void My_uart_restore_irq_My(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief 初始化 USART2 空闲中断 DMA 接收。
  * @details 该函数只负责初始化软件状态并启动第一次接收，
  *          不负责 `USART2` 外设本身的时钟、GPIO、DMA 和中断初始化；
  *          这些底层初始化由 CubeMX 生成的初始化函数完成。
  * @retval HAL 状态
  */
HAL_StatusTypeDef My_uart_init_My(void)
{
  uint8_t index;

  My_uart_rx_queue_head = 0U;
  My_uart_rx_queue_count = 0U;
  for (index = 0U; index < MY_UART_RX_FRAME_QUEUE_SIZE; index++)
  {
    My_uart_rx_frame_lengths[index] = 0U;
  }
  My_uart_rx_overflow = 0U;
  My_uart_rx_restart_error = 0U;

  return My_uart_restart_receive_My();
}

/**
  * @brief 重新启动 USART2 空闲中断 DMA 接收。
  * @details 每次一帧接收完成，或发生错误后，都会调用本函数重新挂起下一轮接收。
  * @retval HAL 状态
  */
HAL_StatusTypeDef My_uart_restart_receive_My(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
                                        My_uart_rx_dma_buffer,
                                        MY_UART_RX_BUFFER_SIZE);
  if (status == HAL_OK && huart2.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }

  return status;
}

/**
  * @brief 使用 USART2 阻塞发送一段数据。
  * @param data 待发送数据的首地址；当 `length > 0` 时不能为空
  * @param length 待发送的数据字节数；传入 `0` 时直接返回 `HAL_OK`
  * @param timeout 发送超时时间，单位为毫秒
  * @retval HAL 状态
  */
HAL_StatusTypeDef My_uart_send_My(const uint8_t *data, uint16_t length, uint32_t timeout)
{
  if (length == 0U)
  {
    return HAL_OK;
  }

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(&huart2, (uint8_t *)data, length, timeout);
}

/**
  * @brief 使用 USART2 阻塞发送字符串。
  * @param text 以 `\\0` 结尾的字符串首地址；不能为空
  * @param timeout 每一段阻塞发送的超时时间，单位为毫秒
  * @retval HAL 状态
  *         - `HAL_OK`：整串字符串发送成功
  *         - 其他返回值：其中某一段发送失败，直接返回对应错误码
  */
HAL_StatusTypeDef My_uart_send_string_My(const char *text, uint32_t timeout)
{
  const uint8_t *cursor;
  size_t remain;

  if (text == NULL)
  {
    return HAL_ERROR;
  }

  cursor = (const uint8_t *)text;
  remain = strlen(text);

  while (remain > 0U)
  {
    uint16_t chunk = (remain > UINT16_MAX) ? UINT16_MAX : (uint16_t)remain;
    HAL_StatusTypeDef status = My_uart_send_My(cursor, chunk, timeout);

    if (status != HAL_OK)
    {
      return status;
    }

    cursor += chunk;
    remain -= chunk;
  }

  return HAL_OK;
}

/**
  * @brief 读取一帧由空闲中断确认的接收数据。
  * @details 该函数在主循环上下文中调用。它会在一个短临界区内把
  *          “四槽环形帧队列”中当前最旧一帧的数据复制到用户缓冲区，
  *          然后释放对应槽位。
  * @param data 用户提供的输出缓冲区首地址
  * @param max_length 用户缓冲区的最大可写长度，单位为字节
  * @retval 实际复制的数据长度
  *         - `0`：没有可读帧或参数非法
  *         - 其他值：本次成功读出的字节数
  */
uint16_t My_uart_read_frame_My(uint8_t *data, uint16_t max_length)
{
  uint32_t primask;
  uint16_t length;
  uint8_t read_index;

  if (data == NULL || max_length == 0U)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (My_uart_rx_queue_count == 0U)
  {
    My_uart_restore_irq_My(primask);
    return 0U;
  }

  read_index = My_uart_rx_queue_head;
  length = My_uart_rx_frame_lengths[read_index];
  if (length > max_length)
  {
    length = max_length;
    My_uart_rx_overflow = 1U;
  }

  memcpy(data, My_uart_rx_frame_queue[read_index], length);
  My_uart_rx_frame_lengths[read_index] = 0U;
  My_uart_rx_queue_head = My_uart_next_queue_index_My(read_index);
  My_uart_rx_queue_count--;

  My_uart_restore_irq_My(primask);

  return length;
}

/**
  * @brief 查询是否有待处理的接收帧。
  * @retval 1 表示有数据，0 表示无数据
  */
uint8_t My_uart_has_frame_My(void)
{
  return (uint8_t)(My_uart_rx_queue_count != 0U);
}

/**
  * @brief 查询接收溢出或重启失败标志。
  * @retval 1 表示发生过异常，0 表示正常
  */
uint8_t My_uart_get_overflow_My(void)
{
  return (uint8_t)((My_uart_rx_overflow != 0U) || (My_uart_rx_restart_error != 0U));
}

/**
  * @brief 清除接收溢出和重启失败标志。
  */
void My_uart_clear_overflow_My(void)
{
  My_uart_rx_overflow = 0U;
  My_uart_rx_restart_error = 0U;
}

/**
  * @brief 在主循环中处理已接收的一帧数据。
  * @details 该函数先尝试读取一帧，再把该帧交给
  *          `My_uart_rx_frame_callback_My()` 做后续业务处理；
  *          若队列中存在多帧数据，则每次只处理最旧的一帧。
  */
void My_uart_poll_My(void)
{
  uint16_t length = My_uart_read_frame_My(My_uart_poll_buffer,
                                                MY_UART_RX_BUFFER_SIZE);
  if (length > 0U)
  {
    My_uart_rx_frame_callback_My(My_uart_poll_buffer, length);
  }
}

/**
  * @brief 接收帧处理回调，默认不执行发送，由用户按需重写。
  * @param data 当前完整接收帧的首地址
  * @param length 当前完整接收帧的有效字节数
  */
__weak void My_uart_rx_frame_callback_My(const uint8_t *data, uint16_t length)
{
  (void)data;
  (void)length;
}

/**
  * @brief USART2 空闲中断 DMA 接收事件回调。
  * @details 该回调运行在中断上下文中，只做“尽快搬运和置位”，
  *          不做耗时业务处理。处理完成后会立即重新启动下一轮接收。
  *          若四槽环形帧队列已满，则会用最新一帧覆盖当前最旧帧，
  *          以优先保证数据实时性。
  * @param huart 触发本次回调的 UART 句柄指针；本实现只处理 `USART2`
  * @param Size 本次从 DMA 缓冲区中判定为“有效接收”的字节数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t copy_length;
  uint8_t write_index;

  if (huart->Instance != USART2)
  {
    return;
  }

  if (Size > 0U)
  {
    copy_length = (Size > MY_UART_RX_BUFFER_SIZE) ? MY_UART_RX_BUFFER_SIZE : Size;
    if (Size > MY_UART_RX_BUFFER_SIZE)
    {
      My_uart_rx_overflow = 1U;
    }

    if (My_uart_rx_queue_count < MY_UART_RX_FRAME_QUEUE_SIZE)
    {
      write_index = (uint8_t)(My_uart_rx_queue_head + My_uart_rx_queue_count);
      if (write_index >= MY_UART_RX_FRAME_QUEUE_SIZE)
      {
        write_index = (uint8_t)(write_index - MY_UART_RX_FRAME_QUEUE_SIZE);
      }

      memcpy(My_uart_rx_frame_queue[write_index], My_uart_rx_dma_buffer, copy_length);
      My_uart_rx_frame_lengths[write_index] = copy_length;
      My_uart_rx_queue_count++;
    }
    else
    {
      write_index = My_uart_rx_queue_head;
      memcpy(My_uart_rx_frame_queue[write_index], My_uart_rx_dma_buffer, copy_length);
      My_uart_rx_frame_lengths[write_index] = copy_length;
      My_uart_rx_queue_head = My_uart_next_queue_index_My(write_index);
      My_uart_rx_overflow = 1U;
    }
  }

  if (My_uart_restart_receive_My() != HAL_OK)
  {
    My_uart_rx_restart_error = 1U;
  }
}

/**
  * @brief USART2 错误回调，发生错误后尝试恢复接收。
  * @param huart 发生错误的 UART 句柄指针；本实现只处理 `USART2`
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  My_uart_rx_overflow = 1U;
  if (My_uart_restart_receive_My() != HAL_OK)
  {
    My_uart_rx_restart_error = 1U;
  }
}

/**
  * @brief 将 printf 的字符输出重定向到 USART2。
  * @param ch 待输出的单个字符
  * @param f 标准库文件流指针
  * @retval 成功时返回原字符，失败时返回 EOF
  */
int fputc(int ch, FILE *f)
{
  uint8_t byte = (uint8_t)ch;

  (void)f;

  if (HAL_UART_Transmit(&huart2, &byte, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    return EOF;
  }

  return ch;
}

/**
  * @brief 标准输入未启用时返回文件结束标志。
  * @param f 标准库文件流指针
  * @retval EOF
  */
int fgetc(FILE *f)
{
  (void)f;
  return EOF;
}

/**
  * @brief 向 ARM 标准库提供终端字符输出接口。
  * @param ch 待输出字符
  */
void _ttywrch(int ch)
{
  (void)fputc(ch, &__stdout);
}

/**
  * @brief 禁止固件退出后进入半主机服务。
  * @param return_code 程序退出码
  */
void _sys_exit(int return_code)
{
  (void)return_code;
  while (1)
  {
  }
}
