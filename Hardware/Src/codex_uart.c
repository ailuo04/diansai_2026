#include "codex_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static uint8_t codex_uart_rx_dma_buffer[CODEX_UART_RX_BUFFER_SIZE];
static uint8_t codex_uart_rx_frame_buffer[CODEX_UART_RX_BUFFER_SIZE];
static uint8_t codex_uart_poll_buffer[CODEX_UART_RX_BUFFER_SIZE];
static volatile uint16_t codex_uart_rx_frame_length;
static volatile uint8_t codex_uart_rx_frame_ready;
static volatile uint8_t codex_uart_rx_overflow;
static volatile uint8_t codex_uart_rx_restart_error;

/**
  * @brief 根据进入临界区前的状态恢复中断。
  * @param primask 进入临界区前的 PRIMASK 值
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
  * @retval HAL 状态
  */
HAL_StatusTypeDef codex_uart_init_codex(void)
{
  codex_uart_rx_frame_length = 0U;
  codex_uart_rx_frame_ready = 0U;
  codex_uart_rx_overflow = 0U;
  codex_uart_rx_restart_error = 0U;

  return codex_uart_restart_receive_codex();
}

/**
  * @brief 重新启动 USART1 空闲中断 DMA 接收。
  * @retval HAL 状态
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
  * @param data 数据指针
  * @param length 数据长度
  * @param timeout 超时时间
  * @retval HAL 状态
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
  * @param text 字符串指针
  * @param timeout 超时时间
  * @retval HAL 状态
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
  * @param data 输出缓冲区
  * @param max_length 输出缓冲区最大长度
  * @retval 实际复制的数据长度
  */
uint16_t codex_uart_read_frame_codex(uint8_t *data, uint16_t max_length)
{
  uint32_t primask;
  uint16_t length;

  if (data == NULL || max_length == 0U)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (codex_uart_rx_frame_ready == 0U)
  {
    codex_uart_restore_irq_codex(primask);
    return 0U;
  }

  length = codex_uart_rx_frame_length;
  if (length > max_length)
  {
    length = max_length;
    codex_uart_rx_overflow = 1U;
  }

  memcpy(data, codex_uart_rx_frame_buffer, length);
  codex_uart_rx_frame_length = 0U;
  codex_uart_rx_frame_ready = 0U;

  codex_uart_restore_irq_codex(primask);

  return length;
}

/**
  * @brief 查询是否有待处理的接收帧。
  * @retval 1 表示有数据，0 表示无数据
  */
uint8_t codex_uart_has_frame_codex(void)
{
  return codex_uart_rx_frame_ready;
}

/**
  * @brief 查询接收溢出或重启失败标志。
  * @retval 1 表示发生过异常，0 表示正常
  */
uint8_t codex_uart_get_overflow_codex(void)
{
  return (uint8_t)((codex_uart_rx_overflow != 0U) || (codex_uart_rx_restart_error != 0U));
}

/**
  * @brief 清除接收溢出和重启失败标志。
  */
void codex_uart_clear_overflow_codex(void)
{
  codex_uart_rx_overflow = 0U;
  codex_uart_rx_restart_error = 0U;
}

/**
  * @brief 在主循环中处理已接收的一帧数据。
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
  * @brief 接收帧处理回调，默认回显收到的数据。
  * @param data 接收数据
  * @param length 接收长度
  */
__weak void codex_uart_rx_frame_callback_codex(const uint8_t *data, uint16_t length)
{
  (void)codex_uart_send_codex(data, length, 100U);
}

/**
  * @brief USART1 空闲中断 DMA 接收事件回调。
  * @param huart UART 句柄
  * @param Size 本次收到的数据长度
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t copy_length;

  if (huart->Instance != USART1)
  {
    return;
  }

  if (Size > 0U)
  {
    copy_length = (Size > CODEX_UART_RX_BUFFER_SIZE) ? CODEX_UART_RX_BUFFER_SIZE : Size;

    if (codex_uart_rx_frame_ready == 0U)
    {
      memcpy(codex_uart_rx_frame_buffer, codex_uart_rx_dma_buffer, copy_length);
      codex_uart_rx_frame_length = copy_length;
      codex_uart_rx_frame_ready = 1U;
    }
    else
    {
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
  * @param huart UART 句柄
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
  * @param ch 待发送字符
  * @param f 标准库文件指针
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
