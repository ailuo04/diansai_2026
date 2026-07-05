#include "codex_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* DMA 正在写入的原始接收缓冲区。 */
static uint8_t codex_uart_rx_dma_buffer[CODEX_UART_RX_BUFFER_SIZE];
/* 由空闲中断确认后的“四槽环形帧队列”。 */
static uint8_t codex_uart_rx_frame_queue[CODEX_UART_RX_FRAME_QUEUE_SIZE][CODEX_UART_RX_BUFFER_SIZE];
/* 每个槽位中当前帧的有效字节数。 */
static volatile uint16_t codex_uart_rx_frame_lengths[CODEX_UART_RX_FRAME_QUEUE_SIZE];
/* 指向当前最旧未读帧的槽位索引。 */
static volatile uint8_t codex_uart_rx_queue_head;
/* 当前队列中待处理帧的数量，范围为 0~4。 */
static volatile uint8_t codex_uart_rx_queue_count;
/* 主循环读取最旧一帧时使用的临时拷贝缓冲区。 */
static uint8_t codex_uart_poll_buffer[CODEX_UART_RX_BUFFER_SIZE];
/* 接收过程中是否发生过队列覆盖、缓冲区截断或恢复异常。 */
static volatile uint8_t codex_uart_rx_overflow;
/* 重新启动 DMA 接收流程时是否发生过失败。 */
static volatile uint8_t codex_uart_rx_restart_error;

/**
  * @brief 计算环形队列中的下一个槽位索引。
  * @param index 当前槽位索引
  * @retval 下一个槽位索引；到达队尾后会自动回绕到 `0`
  */
static uint8_t codex_uart_next_queue_index_codex(uint8_t index)
{
  index++;
  if (index >= CODEX_UART_RX_FRAME_QUEUE_SIZE)
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
static void codex_uart_restore_irq_codex(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief 初始化 USART1 空闲中断 DMA 接收。
  * @details 该函数只负责初始化软件状态并启动第一次接收，
  *          不负责 `USART1` 外设本身的时钟、GPIO、DMA 和中断初始化；
  *          这些底层初始化由 `MX_USART1_UART_Init()` 完成。
  * @retval HAL 状态
  *         - `HAL_OK`：首次接收启动成功
  *         - `HAL_ERROR/HAL_BUSY/HAL_TIMEOUT`：首次接收启动失败
  */
HAL_StatusTypeDef codex_uart_init_codex(void)
{
  uint8_t index;

  codex_uart_rx_queue_head = 0U;
  codex_uart_rx_queue_count = 0U;
  for (index = 0U; index < CODEX_UART_RX_FRAME_QUEUE_SIZE; index++)
  {
    codex_uart_rx_frame_lengths[index] = 0U;
  }
  codex_uart_rx_overflow = 0U;
  codex_uart_rx_restart_error = 0U;

  return codex_uart_restart_receive_codex();
}

/**
  * @brief 重新启动 USART1 空闲中断 DMA 接收。
  * @details 每次一帧接收完成，或发生错误后，都会调用本函数重新挂起下一轮接收。
  * @retval HAL 状态
  *         - `HAL_OK`：重启成功
  *         - `HAL_ERROR/HAL_BUSY/HAL_TIMEOUT`：重启失败
  */
HAL_StatusTypeDef codex_uart_restart_receive_codex(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                        codex_uart_rx_dma_buffer,
                                        CODEX_UART_RX_BUFFER_SIZE);
  if (status == HAL_OK && huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }

  return status;
}

/**
  * @brief 使用 USART1 阻塞发送一段数据。
  * @param data 待发送数据的首地址；当 `length > 0` 时不能为空
  * @param length 待发送的数据字节数；传入 `0` 时直接返回 `HAL_OK`
  * @param timeout 发送超时时间，单位为毫秒；实际由 `HAL_UART_Transmit()` 解释
  * @retval HAL 状态
  *         - `HAL_OK`：发送成功
  *         - `HAL_ERROR`：空指针等非法参数，或 HAL 发送失败
  *         - `HAL_BUSY/HAL_TIMEOUT`：串口忙或发送超时
  */
HAL_StatusTypeDef codex_uart_send_codex(const uint8_t *data, uint16_t length, uint32_t timeout)
{
  if (length == 0U)
  {
    return HAL_OK;
  }

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout);
}

/**
  * @brief 使用 USART1 阻塞发送字符串。
  * @param text 以 `\\0` 结尾的字符串首地址；不能为空
  * @param timeout 每一段阻塞发送的超时时间，单位为毫秒
  * @retval HAL 状态
  *         - `HAL_OK`：整串字符串发送成功
  *         - 其他返回值：其中某一段发送失败，直接返回对应错误码
  */
HAL_StatusTypeDef codex_uart_send_string_codex(const char *text, uint32_t timeout)
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
    HAL_StatusTypeDef status = codex_uart_send_codex(cursor, chunk, timeout);

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
uint16_t codex_uart_read_frame_codex(uint8_t *data, uint16_t max_length)
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

  if (codex_uart_rx_queue_count == 0U)
  {
    codex_uart_restore_irq_codex(primask);
    return 0U;
  }

  read_index = codex_uart_rx_queue_head;
  length = codex_uart_rx_frame_lengths[read_index];
  if (length > max_length)
  {
    length = max_length;
    codex_uart_rx_overflow = 1U;
  }

  memcpy(data, codex_uart_rx_frame_queue[read_index], length);
  codex_uart_rx_frame_lengths[read_index] = 0U;
  codex_uart_rx_queue_head = codex_uart_next_queue_index_codex(read_index);
  codex_uart_rx_queue_count--;

  codex_uart_restore_irq_codex(primask);

  return length;
}

/**
  * @brief 查询是否有待处理的接收帧。
  * @details 这是一个轻量级状态查询接口，不会清除任何标志；
  *          只要环形帧队列非空，就会返回 `1`。
  * @retval 1 表示有数据，0 表示无数据
  */
uint8_t codex_uart_has_frame_codex(void)
{
  return (uint8_t)(codex_uart_rx_queue_count != 0U);
}

/**
  * @brief 查询接收溢出或重启失败标志。
  * @details 只要发生过以下任意一种情况，本函数就会返回 `1`：
  *          1. 四槽环形帧队列已满时，新帧覆盖了最旧帧；
  *          2. 读取帧时用户缓冲区长度不足而被截断；
  *          3. 回调判定出的接收长度超过单帧缓冲区而被截断；
  *          4. 重新启动 DMA 接收失败。
  * @retval 1 表示发生过异常，0 表示正常
  */
uint8_t codex_uart_get_overflow_codex(void)
{
  return (uint8_t)((codex_uart_rx_overflow != 0U) || (codex_uart_rx_restart_error != 0U));
}

/**
  * @brief 清除接收溢出和重启失败标志。
  * @details 一般在用户已经记录或处理完异常后调用。
  */
void codex_uart_clear_overflow_codex(void)
{
  codex_uart_rx_overflow = 0U;
  codex_uart_rx_restart_error = 0U;
}

/**
  * @brief 在主循环中处理已接收的一帧数据。
  * @details 该函数先尝试读取一帧，再把该帧交给
  *          `codex_uart_rx_frame_callback_codex()` 做后续业务处理；
  *          若队列中存在多帧数据，则每次只处理最旧的一帧。
  */
void codex_uart_poll_codex(void)
{
  uint16_t length = codex_uart_read_frame_codex(codex_uart_poll_buffer,
                                                CODEX_UART_RX_BUFFER_SIZE);
  if (length > 0U)
  {
    codex_uart_rx_frame_callback_codex(codex_uart_poll_buffer, length);
  }
}

/**
  * @brief 接收帧处理回调，默认不执行发送，由用户按需重写。
  * @details 这是一个弱定义函数。若用户在其他源文件里实现同名强定义函数，
  *          则链接时会覆盖本默认实现。
  * @param data 当前完整接收帧的首地址
  * @param length 当前完整接收帧的有效字节数
  */
__weak void codex_uart_rx_frame_callback_codex(const uint8_t *data, uint16_t length)
{
  (void)data;
  (void)length;
}

/**
  * @brief USART1 空闲中断 DMA 接收事件回调。
  * @details 该回调运行在中断上下文中，只做“尽快搬运和置位”，
  *          不做耗时业务处理。处理完成后会立即重新启动下一轮接收。
  *          若四槽环形帧队列已满，则会用最新一帧覆盖当前最旧帧，
  *          以优先保证数据实时性。
  * @param huart 触发本次回调的 UART 句柄指针；本实现只处理 `USART1`
  * @param Size 本次从 DMA 缓冲区中判定为“有效接收”的字节数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t copy_length;
  uint8_t write_index;

  if (huart->Instance != USART1)
  {
    return;
  }

  if (Size > 0U)
  {
    copy_length = (Size > CODEX_UART_RX_BUFFER_SIZE) ? CODEX_UART_RX_BUFFER_SIZE : Size;
    if (Size > CODEX_UART_RX_BUFFER_SIZE)
    {
      codex_uart_rx_overflow = 1U;
    }

    if (codex_uart_rx_queue_count < CODEX_UART_RX_FRAME_QUEUE_SIZE)
    {
      write_index = (uint8_t)(codex_uart_rx_queue_head + codex_uart_rx_queue_count);
      if (write_index >= CODEX_UART_RX_FRAME_QUEUE_SIZE)
      {
        write_index = (uint8_t)(write_index - CODEX_UART_RX_FRAME_QUEUE_SIZE);
      }

      memcpy(codex_uart_rx_frame_queue[write_index], codex_uart_rx_dma_buffer, copy_length);
      codex_uart_rx_frame_lengths[write_index] = copy_length;
      codex_uart_rx_queue_count++;
    }
    else
    {
      write_index = codex_uart_rx_queue_head;
      memcpy(codex_uart_rx_frame_queue[write_index], codex_uart_rx_dma_buffer, copy_length);
      codex_uart_rx_frame_lengths[write_index] = copy_length;
      codex_uart_rx_queue_head = codex_uart_next_queue_index_codex(write_index);
      codex_uart_rx_overflow = 1U;
    }
  }

  if (codex_uart_restart_receive_codex() != HAL_OK)
  {
    codex_uart_rx_restart_error = 1U;
  }
}

/**
  * @brief USART1 错误回调，发生错误后尝试恢复接收。
  * @details 该回调运行在中断相关上下文中。当前实现把错误统一视为异常，
  *          先置位异常标志，再尝试恢复下一轮 DMA 接收。
  * @param huart 发生错误的 UART 句柄指针；本实现只处理 `USART1`
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  codex_uart_rx_overflow = 1U;
  if (codex_uart_restart_receive_codex() != HAL_OK)
  {
    codex_uart_rx_restart_error = 1U;
  }
}

/**
  * @brief 将 printf 的字符输出重定向到 USART1。
  * @details 这是标准库字符输出钩子。当前实现是“单字符阻塞发送”，
  *          因此 `printf` 输出过多时会占用较长主循环时间。
  * @param ch 待输出的单个字符，取值范围由 C 标准库 `int` 约定
  * @param f 标准库文件流指针；当前实现未使用该参数，仅为满足函数签名保留
  * @retval 成功时返回原字符，失败时返回 EOF
  */
int fputc(int ch, FILE *f)
{
  uint8_t byte = (uint8_t)ch;

  (void)f;

  if (HAL_UART_Transmit(&huart1, &byte, 1U, HAL_MAX_DELAY) != HAL_OK)
  {
    return EOF;
  }

  return ch;
}
