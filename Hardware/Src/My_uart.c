#include "My_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <rt_misc.h>

__asm(".global __use_no_semihosting\n");

/* ARM 标准库需要由应用提供文件流对象，避免启动阶段进入半主机断点。 */
FILE __stdout; /* ARM 标准库 printf 使用的标准输出文件流占位对象。 */
FILE __stdin;  /* ARM 标准库标准输入使用的文件流占位对象，本工程不接收标准输入。 */

/* DMA 正在写入的原始接收缓冲区。 */
static uint8_t My_uart_rx_dma_buffer[MY_UART_RX_BUFFER_SIZE]; /* USART3 DMA 当前正在写入的原始接收缓冲区。 */
/* 由空闲中断确认后的“四槽环形帧队列”。 */
static uint8_t My_uart_rx_frame_queue[MY_UART_RX_FRAME_QUEUE_SIZE][MY_UART_RX_BUFFER_SIZE]; /* 空闲中断确认后的完整接收帧环形队列。 */
/* 每个槽位中当前帧的有效字节数。 */
static volatile uint16_t My_uart_rx_frame_lengths[MY_UART_RX_FRAME_QUEUE_SIZE]; /* 各接收队列槽位当前保存的有效帧长度。 */
/* 指向当前最旧未读帧的槽位索引。 */
static volatile uint8_t My_uart_rx_queue_head; /* 当前最旧未读接收帧所在的队列槽位索引。 */
/* 当前队列中待处理帧的数量，范围为 0~4。 */
static volatile uint8_t My_uart_rx_queue_count; /* 当前接收队列中等待主循环处理的帧数量。 */
/* 主循环读取最旧一帧时使用的临时拷贝缓冲区。 */
static uint8_t My_uart_poll_buffer[MY_UART_RX_BUFFER_SIZE]; /* 主循环取出一帧后交给业务回调的临时缓冲区。 */
/* 接收过程中是否发生过队列覆盖、缓冲区截断或恢复异常。 */
static volatile uint8_t My_uart_rx_overflow; /* 非零表示发生队列覆盖、帧截断或其他接收异常。 */
/* 重新启动 DMA 接收流程时是否发生过失败。 */
static volatile uint8_t My_uart_rx_restart_error; /* 非零表示重新挂载 USART3 DMA 接收失败。 */
/* DMA 发送环形缓冲区，调用者数据在返回前会复制到此处。 */
static uint8_t My_uart_tx_buffer[MY_UART_TX_BUFFER_SIZE]; /* 保存调用者待发送数据的 DMA 发送环形缓冲区。 */
/* 当前最旧待发送字节在环形缓冲区中的索引。 */
static volatile uint16_t My_uart_tx_head; /* 当前最旧待发送字节在发送环形缓冲区中的索引。 */
/* 环形缓冲区中等待发送及正在发送的总字节数。 */
static volatile uint16_t My_uart_tx_count; /* 发送环形缓冲区中等待发送及正在发送的总字节数。 */
/* 当前 DMA 正在发送的连续字节数，0 表示 DMA 空闲。 */
static volatile uint16_t My_uart_tx_dma_length; /* 当前一次 DMA 连续发送区域的字节数，0 表示空闲。 */
/* DMA 发送启动或运行过程中是否发生过异常。 */
static volatile uint8_t My_uart_tx_error; /* 非零表示 DMA 发送启动、完成或状态一致性发生异常。 */

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
  * @brief 在发送队列非空且 DMA 空闲时启动一次连续区域发送。
  * @retval HAL 状态；队列为空或已有发送进行时返回 `HAL_OK`
  */
static HAL_StatusTypeDef My_uart_start_transmit_My(void)
{
  HAL_StatusTypeDef status; /* 本次启动 USART3 DMA 发送接口返回的状态。 */
  uint32_t primask; /* 检查和更新发送队列前保存的全局中断屏蔽状态。 */
  uint16_t length; /* 本次可从发送环形缓冲区连续交给 DMA 的字节数。 */

  primask = __get_PRIMASK();
  __disable_irq();

  if (My_uart_tx_dma_length != 0U || My_uart_tx_count == 0U)
  {
    My_uart_restore_irq_My(primask);
    return HAL_OK;
  }

  length = My_uart_tx_count;
  if (length > (uint16_t)(MY_UART_TX_BUFFER_SIZE - My_uart_tx_head))
  {
    length = (uint16_t)(MY_UART_TX_BUFFER_SIZE - My_uart_tx_head);
  }

  My_uart_tx_dma_length = length;
  status = HAL_UART_Transmit_DMA(&huart3,
                                 &My_uart_tx_buffer[My_uart_tx_head],
                                 length);
  if (status != HAL_OK)
  {
    My_uart_tx_dma_length = 0U;
    if (status != HAL_BUSY)
    {
      My_uart_tx_error = 1U;
    }
  }

  My_uart_restore_irq_My(primask);
  return status;
}

/**
  * @brief 初始化 USART3 DMA 收发。
  * @details 该函数只负责初始化软件状态并启动第一次接收，
  *          不负责 `USART3` 外设本身的时钟、GPIO、DMA 和中断初始化；
  *          这些底层初始化由 CubeMX 生成的初始化函数完成。
  * @retval HAL 状态
  */
HAL_StatusTypeDef My_uart_init_My(void)
{
  uint8_t index; /* 初始化时当前清零的接收帧队列槽位索引。 */

  My_uart_rx_queue_head = 0U;
  My_uart_rx_queue_count = 0U;
  for (index = 0U; index < MY_UART_RX_FRAME_QUEUE_SIZE; index++)
  {
    My_uart_rx_frame_lengths[index] = 0U;
  }
  My_uart_rx_overflow = 0U;
  My_uart_rx_restart_error = 0U;
  My_uart_tx_head = 0U;
  My_uart_tx_count = 0U;
  My_uart_tx_dma_length = 0U;
  My_uart_tx_error = 0U;

  return My_uart_restart_receive_My();
}

/**
  * @brief 重新启动 USART3 空闲中断 DMA 接收。
  * @details 每次一帧接收完成，或发生错误后，都会调用本函数重新挂起下一轮接收。
  * @retval HAL 状态
  */
HAL_StatusTypeDef My_uart_restart_receive_My(void)
{
  HAL_StatusTypeDef status; /* 重新挂载 USART3 空闲中断 DMA 接收的返回状态。 */

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3,
                                        My_uart_rx_dma_buffer,
                                        MY_UART_RX_BUFFER_SIZE);
  if (status == HAL_OK && huart3.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }

  return status;
}

/**
  * @brief 把一段数据复制到 USART3 DMA 发送队列。
  * @param data 待发送数据的首地址；当 `length > 0` 时不能为空
  * @param length 待发送的数据字节数；传入 `0` 时直接返回 `HAL_OK`
  * @param timeout 等待发送队列空间的超时时间，单位为毫秒
  * @retval HAL 状态
  */
HAL_StatusTypeDef My_uart_send_My(const uint8_t *data, uint16_t length, uint32_t timeout)
{
  HAL_StatusTypeDef status; /* 当前一次发送队列启动或推进操作的返回状态。 */
  uint32_t start_tick; /* 本次入队等待开始时的系统毫秒时刻，用于超时判断。 */
  uint16_t sent_length = 0U; /* 调用者数据已经复制进发送环形缓冲区的字节数。 */

  if (length == 0U)
  {
    return HAL_OK;
  }

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (sent_length < length)
  {
    uint32_t primask; /* 本轮检查发送缓冲区空间前保存的全局中断屏蔽状态。 */
    uint16_t available; /* 本轮发送环形缓冲区剩余的可写字节数。 */
    uint16_t copy_length; /* 本轮实际复制到连续可写区域的数据长度。 */
    uint16_t write_index; /* 本轮数据在发送环形缓冲区中的起始写入索引。 */

    primask = __get_PRIMASK();
    __disable_irq();

    available = (uint16_t)(MY_UART_TX_BUFFER_SIZE - My_uart_tx_count);
    if (available > 0U)
    {
      write_index = (uint16_t)(My_uart_tx_head + My_uart_tx_count);
      if (write_index >= MY_UART_TX_BUFFER_SIZE)
      {
        write_index = (uint16_t)(write_index - MY_UART_TX_BUFFER_SIZE);
      }

      copy_length = (uint16_t)(length - sent_length);
      if (copy_length > available)
      {
        copy_length = available;
      }
      if (copy_length > (uint16_t)(MY_UART_TX_BUFFER_SIZE - write_index))
      {
        copy_length = (uint16_t)(MY_UART_TX_BUFFER_SIZE - write_index);
      }

      memcpy(&My_uart_tx_buffer[write_index], &data[sent_length], copy_length);
      My_uart_tx_count = (uint16_t)(My_uart_tx_count + copy_length);
      sent_length = (uint16_t)(sent_length + copy_length);
    }

    My_uart_restore_irq_My(primask);

    status = My_uart_start_transmit_My();
    if (status != HAL_OK && status != HAL_BUSY)
    {
      return status;
    }

    if (sent_length < length && timeout != HAL_MAX_DELAY &&
        (HAL_GetTick() - start_tick) >= timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @brief 把字符串复制到 USART3 DMA 发送队列。
  * @param text 以 `\\0` 结尾的字符串首地址；不能为空
  * @param timeout 每一段阻塞发送的超时时间，单位为毫秒
  * @retval HAL 状态
  *         - `HAL_OK`：整串字符串发送成功
  *         - 其他返回值：其中某一段发送失败，直接返回对应错误码
  */
HAL_StatusTypeDef My_uart_send_string_My(const char *text, uint32_t timeout)
{
  const uint8_t *cursor; /* 当前尚未加入发送队列的字符串数据起始地址。 */
  size_t remain; /* 当前尚未加入发送队列的字符串字节数。 */

  if (text == NULL)
  {
    return HAL_ERROR;
  }

  cursor = (const uint8_t *)text;
  remain = strlen(text);

  while (remain > 0U)
  {
    uint16_t chunk = (remain > UINT16_MAX) ? UINT16_MAX : (uint16_t)remain; /* 受底层 16 位长度接口限制的本轮发送字节数。 */
    HAL_StatusTypeDef status = My_uart_send_My(cursor, chunk, timeout); /* 本轮字符串分段加入发送队列的结果。 */

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
  uint32_t primask; /* 读取和释放接收帧队列槽位前保存的全局中断屏蔽状态。 */
  uint16_t length; /* 当前最旧接收帧经调用者缓冲区限幅后的复制长度。 */
  uint8_t read_index; /* 当前最旧未读接收帧所在的队列槽位索引。 */

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
  return (uint8_t)((My_uart_rx_overflow != 0U) ||
                   (My_uart_rx_restart_error != 0U) ||
                   (My_uart_tx_error != 0U));
}

/**
  * @brief 清除接收溢出和重启失败标志。
  */
void My_uart_clear_overflow_My(void)
{
  My_uart_rx_overflow = 0U;
  My_uart_rx_restart_error = 0U;
  My_uart_tx_error = 0U;
}

/**
  * @brief 在主循环中处理已接收的一帧数据。
  * @details 该函数先尝试读取一帧，再把该帧交给
  *          `My_uart_rx_frame_callback_My()` 做后续业务处理；
  *          若队列中存在多帧数据，则每次只处理最旧的一帧。
  */
void My_uart_poll_My(void)
{
  HAL_StatusTypeDef status = My_uart_start_transmit_My(); /* 主循环本轮推进 DMA 发送队列的结果。 */
  uint16_t length = My_uart_read_frame_My(My_uart_poll_buffer, /* 主循环本轮从接收队列取出的完整帧长度。 */
                                                MY_UART_RX_BUFFER_SIZE);

  if (status != HAL_OK && status != HAL_BUSY)
  {
    My_uart_tx_error = 1U;
  }
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
  * @brief USART3 空闲中断 DMA 接收事件回调。
  * @details 该回调运行在中断上下文中，只做“尽快搬运和置位”，
  *          不做耗时业务处理。处理完成后会立即重新启动下一轮接收。
  *          若四槽环形帧队列已满，则会用最新一帧覆盖当前最旧帧，
  *          以优先保证数据实时性。该函数与主循环共享帧队列计数、队首和
  *          溢出标志，这些字段必须保持 volatile 且仅执行自然宽度原子访问。
  *          中断内禁止等待、打印、动态分配或直接解析业务协议。
  * @param huart 触发本次回调的 UART 句柄指针；本实现只处理 `USART3`
  * @param Size 本次从 DMA 缓冲区中判定为“有效接收”的字节数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t copy_length; /* 本次 DMA 接收事件允许复制进帧队列的有效字节数。 */
  uint8_t write_index; /* 本次新帧写入或覆盖的接收队列槽位索引。 */

  /* HAL 回调为所有 UART 共用；非 USART3 事件不属于本驱动，必须无副作用返回。 */
  if (huart->Instance != USART3)
  {
    return;
  }

  /* Size 为本轮 DMA 接收到的有效长度；空事件不入队，但仍需在末尾重启接收。 */
  if (Size > 0U)
  {
    /* 限制复制长度，防止异常 Size 越过 DMA 缓冲区，并锁存数据截断诊断标志。 */
    copy_length = (Size > MY_UART_RX_BUFFER_SIZE) ? MY_UART_RX_BUFFER_SIZE : Size;
    if (Size > MY_UART_RX_BUFFER_SIZE)
    {
      My_uart_rx_overflow = 1U;
    }

    /* 队列未满时在队尾写入，新帧写完长度后再增加计数，避免主循环读到半帧。 */
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
      /*
       * 队列已满时覆盖最旧帧并推进队首，队列数量保持不变；该策略会丢弃
       * 一帧历史数据，因此同时置位溢出标志供主循环诊断。
       */
      write_index = My_uart_rx_queue_head;
      memcpy(My_uart_rx_frame_queue[write_index], My_uart_rx_dma_buffer, copy_length);
      My_uart_rx_frame_lengths[write_index] = copy_length;
      My_uart_rx_queue_head = My_uart_next_queue_index_My(write_index);
      My_uart_rx_overflow = 1U;
    }
  }

  /* 每次事件后重新挂载 DMA 空闲接收；失败仅锁存标志，等待主循环决定恢复策略。 */
  if (My_uart_restart_receive_My() != HAL_OK)
  {
    My_uart_rx_restart_error = 1U;
  }
}

/**
  * @brief USART3 DMA 发送完成回调。
  * @details 释放本次已发送区域，并立即启动环形缓冲区中的下一段数据。
  *          本函数与主循环共享发送队首、待发计数和 DMA 长度，更新时短暂
  *          关闭中断形成临界区；禁止在临界区内等待、打印或调用阻塞接口。
  * @param huart 触发本次回调的 UART 句柄指针
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask; /* 更新发送队列共享状态前保存的全局中断屏蔽状态。 */
  uint16_t completed_length; /* 刚完成的 USART3 DMA 发送区域字节数。 */

  /* 忽略其他 UART 的发送完成事件，防止误推进 USART3 环形缓冲区。 */
  if (huart->Instance != USART3)
  {
    return;
  }

  /* 保存进入回调前的全局中断状态，临界区结束后按原状态恢复，避免误开启中断。 */
  primask = __get_PRIMASK();
  __disable_irq();

  /*
   * 读取本次 DMA 长度并释放对应队首区域。正常路径推进队首、减少待发计数；
   * 若长度超过队列计数，说明状态损坏，清空队列并锁存错误以阻止越界访问。
   */
  completed_length = My_uart_tx_dma_length;
  if (completed_length > 0U && completed_length <= My_uart_tx_count)
  {
    My_uart_tx_head = (uint16_t)(My_uart_tx_head + completed_length);
    if (My_uart_tx_head >= MY_UART_TX_BUFFER_SIZE)
    {
      My_uart_tx_head = (uint16_t)(My_uart_tx_head - MY_UART_TX_BUFFER_SIZE);
    }
    My_uart_tx_count = (uint16_t)(My_uart_tx_count - completed_length);
  }
  else if (completed_length != 0U)
  {
    My_uart_tx_head = 0U;
    My_uart_tx_count = 0U;
    My_uart_tx_error = 1U;
  }
  My_uart_tx_dma_length = 0U;

  /* 环形队列状态已一致，恢复原中断状态后再启动下一段，缩短全局关中断时间。 */
  My_uart_restore_irq_My(primask);
  /* 非阻塞启动下一段连续数据；失败时锁存错误，由后续轮询继续尝试恢复。 */
  if (My_uart_start_transmit_My() != HAL_OK)
  {
    My_uart_tx_error = 1U;
  }
}

/**
  * @brief USART3 错误回调，发生错误后尝试恢复 DMA 收发。
  * @details 回调运行于中断上下文，只复位必要状态并重新挂载非阻塞 DMA。
  *          它会修改与主循环共享的收发错误标志和当前 DMA 长度；禁止在此处
  *          打印错误、延时或执行协议解析，否则会延长 USART3 中断占用时间。
  * @param huart 发生错误的 UART 句柄指针；本实现只处理 `USART3`
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* HAL 错误回调为所有 UART 共用，非 USART3 错误不改变本驱动状态。 */
  if (huart->Instance != USART3)
  {
    return;
  }

  /* 统一锁存接收异常，提示本轮数据可能不完整或已经丢失。 */
  My_uart_rx_overflow = 1U;
  /*
   * 若 HAL 已回到发送就绪态但软件仍记录在途 DMA，说明该段发送被错误中止；
   * 清除在途长度并重新启动队首数据，保留待发计数以避免静默丢包。
   */
  if (My_uart_tx_dma_length != 0U && huart->gState == HAL_UART_STATE_READY)
  {
    My_uart_tx_dma_length = 0U;
    My_uart_tx_error = 1U;
    if (My_uart_start_transmit_My() != HAL_OK)
    {
      My_uart_tx_error = 1U;
    }
  }
  /* 无论错误来源为何都重新挂载接收 DMA；失败时锁存恢复错误供主循环诊断。 */
  if (My_uart_restart_receive_My() != HAL_OK)
  {
    My_uart_rx_restart_error = 1U;
  }
}

/**
  * @brief 将 printf 的字符复制到 USART3 DMA 发送队列。
  * @param ch 待输出的单个字符
  * @param f 标准库文件流指针
  * @retval 成功时返回原字符，失败时返回 EOF
  */
int fputc(int ch, FILE *f)
{
  uint8_t byte = (uint8_t)ch; /* 转换后准备加入 USART3 发送队列的单字节数据。 */

  (void)f;

  if (My_uart_send_My(&byte, 1U, HAL_MAX_DELAY) != HAL_OK)
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
