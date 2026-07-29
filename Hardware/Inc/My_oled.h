#ifndef MY_OLED_H
#define MY_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* SSD1306 常见 128x64 模块的两个 7 位 I2C 地址。 */
#define MY_OLED_I2C_ADDRESS_LOW  0x3CU
#define MY_OLED_I2C_ADDRESS_HIGH 0x3DU

/**
  * @brief 初始化 OLED，并自动探测 0x3C 和 0x3D 地址。
  * @retval HAL_OK 表示 OLED 已应答并完成初始化，HAL_ERROR 表示总线或器件无应答
  */
HAL_StatusTypeDef My_oled_init_My(void);

/**
  * @brief 获取初始化时探测到的 OLED 7 位地址。
  * @retval 0x3C、0x3D，未探测到器件时返回 0
  */
uint8_t My_oled_get_address_My(void);
void My_oled_clear_My(void);
void My_oled_set_cursor_My(uint8_t page, uint8_t column);
void My_oled_write_char_My(char value);
void My_oled_write_string_My(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* MY_OLED_H */
