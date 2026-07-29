#ifndef MY_UART_H
#define MY_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define MY_UART_RX_BUFFER_SIZE 256U
#define MY_UART_RX_FRAME_QUEUE_SIZE 4U
#define MY_UART_TX_BUFFER_SIZE 512U

/**
  * @brief 初始化 USART3 DMA 收发。
  * @details 该函数会清零当前串口收发状态，并启动一次新的
  *          `ReceiveToIdle + DMA` 接收流程。
  * @retval HAL 状态
  *         - `HAL_OK`：启动成功
  *         - `HAL_ERROR/HAL_BUSY/HAL_TIMEOUT`：启动失败，具体原因由 HAL 提供
  */
HAL_StatusTypeDef My_uart_init_My(void);

/**
  * @brief 重新启动 USART3 空闲中断 DMA 接收。
  * @details 该函数会把 `USART3` 的 DMA 接收缓冲区重新挂到 HAL，
  *          并关闭 DMA 半传输中断，避免半包时提前进入回调。
  * @retval HAL 状态
  *         - `HAL_OK`：重启成功
  *         - `HAL_ERROR/HAL_BUSY/HAL_TIMEOUT`：重启失败，具体原因由 HAL 提供
  */
HAL_StatusTypeDef My_uart_restart_receive_My(void);

/**
  * @brief 把一段数据复制到 USART3 DMA 发送队列。
  * @param data 待发送数据的首地址；当 `length > 0` 时不能为空指针
  * @param length 待发送的字节数，单位为字节；传入 `0` 时函数直接返回成功
  * @param timeout 等待发送队列空间的超时时间，单位为毫秒
  * @retval HAL 状态
  *         - `HAL_OK`：数据已全部进入 DMA 发送队列
  *         - `HAL_ERROR`：参数非法或 DMA 启动失败
  *         - `HAL_TIMEOUT`：等待发送队列空间超时
  */
HAL_StatusTypeDef My_uart_send_My(const uint8_t *data, uint16_t length, uint32_t timeout);

/**
  * @brief 把字符串复制到 USART3 DMA 发送队列。
  * @param text 以 `\\0` 结尾的字符串首地址；不能为空指针
  * @param timeout 每一段字符串等待发送队列空间的超时时间，单位为毫秒
  * @retval HAL 状态
  *         - `HAL_OK`：整串字符串已进入 DMA 发送队列
  *         - 其他返回值：任意一段发送失败时直接返回对应错误码
  */
HAL_StatusTypeDef My_uart_send_string_My(const char *text, uint32_t timeout);

/**
  * @brief 读取一帧由空闲中断确认的接收数据。
  * @details 当存在多帧待处理数据时，本函数总是按接收先后顺序，
  *          读取当前最旧的一帧。
  * @param data 用户提供的输出缓冲区首地址，用于接收拷出的完整一帧数据
  * @param max_length 输出缓冲区最大可写长度，单位为字节；用于防止拷贝越界
  * @retval 实际复制的数据长度
  *         - `0`：当前没有可读帧，或参数非法
  *         - 其他值：本次成功读出的帧长度
  */
uint16_t My_uart_read_frame_My(uint8_t *data, uint16_t max_length);

/**
  * @brief 查询是否有待处理的接收帧。
  * @details 只要四槽环形帧队列中至少存在一帧未读数据，本函数就返回 `1`。
  * @retval 1 表示有数据，0 表示无数据
  */
uint8_t My_uart_has_frame_My(void);

/**
  * @brief 查询接收溢出或重启失败标志。
  * @details 只要发生过以下任意一种情况，本函数就会返回 `1`：
  *          1. 四槽环形帧队列已满时，新帧覆盖了最旧帧；
  *          2. 读取帧时用户缓冲区长度不足而被截断；
  *          3. 回调判定出的接收长度超过单帧缓冲区而被截断；
  *          4. 重新启动 DMA 接收失败；
  *          5. DMA 发送启动或运行异常。
  * @retval 1 表示发生过异常，0 表示正常
  */
uint8_t My_uart_get_overflow_My(void);

/**
  * @brief 清除接收溢出和重启失败标志。
  * @details 一般在用户已经记录或处理完异常后调用。
  */
void My_uart_clear_overflow_My(void);

/**
  * @brief 在主循环中处理已接收的一帧数据。
  * @details 当队列中存在多帧数据时，本函数每次只取出并处理当前最旧的一帧。
  */
void My_uart_poll_My(void);

/**
  * @brief 接收帧处理回调，默认不执行发送，由用户按需重写。
  * @details 该回调由 `My_uart_poll_My()` 在主循环上下文中调用，
  *          因此适合放业务解析逻辑，不建议在中断里直接处理的工作也应放到这里。
  * @param data 当前完整接收帧的首地址，只在本次回调期间有效
  * @param length 当前完整接收帧的有效字节数
  */
void My_uart_rx_frame_callback_My(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* MY_UART_H */
