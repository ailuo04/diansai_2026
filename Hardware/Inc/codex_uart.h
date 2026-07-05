#ifndef CODEX_UART_H
#define CODEX_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define CODEX_UART_RX_BUFFER_SIZE 256U

/**
  * @brief 初始化 USART1 空闲中断 DMA 接收。
  * @retval HAL 状态
  */
HAL_StatusTypeDef codex_uart_init_codex(void);

/**
  * @brief 重新启动 USART1 空闲中断 DMA 接收。
  * @retval HAL 状态
  */
HAL_StatusTypeDef codex_uart_restart_receive_codex(void);

/**
  * @brief 使用 USART1 阻塞发送一段数据。
  * @param data 数据指针
  * @param length 数据长度
  * @param timeout 超时时间
  * @retval HAL 状态
  */
HAL_StatusTypeDef codex_uart_send_codex(const uint8_t *data, uint16_t length, uint32_t timeout);

/**
  * @brief 使用 USART1 阻塞发送字符串。
  * @param text 字符串指针
  * @param timeout 超时时间
  * @retval HAL 状态
  */
HAL_StatusTypeDef codex_uart_send_string_codex(const char *text, uint32_t timeout);

/**
  * @brief 读取一帧由空闲中断确认的接收数据。
  * @param data 输出缓冲区
  * @param max_length 输出缓冲区最大长度
  * @retval 实际复制的数据长度
  */
uint16_t codex_uart_read_frame_codex(uint8_t *data, uint16_t max_length);

/**
  * @brief 查询是否有待处理的接收帧。
  * @retval 1 表示有数据，0 表示无数据
  */
uint8_t codex_uart_has_frame_codex(void);

/**
  * @brief 查询接收溢出或重启失败标志。
  * @retval 1 表示发生过异常，0 表示正常
  */
uint8_t codex_uart_get_overflow_codex(void);

/**
  * @brief 清除接收溢出和重启失败标志。
  */
void codex_uart_clear_overflow_codex(void);

/**
  * @brief 在主循环中处理已接收的一帧数据。
  */
void codex_uart_poll_codex(void);

/**
  * @brief 接收帧处理回调，默认回显收到的数据。
  * @param data 接收数据
  * @param length 接收长度
  */
void codex_uart_rx_frame_callback_codex(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* CODEX_UART_H */
