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
#define MY_OLED_ROW_COUNT              4U
#define MY_OLED_CHARACTER_COLUMN_COUNT 16U
#define MY_OLED_CHARACTER_WIDTH        8U

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

/**
  * @brief 从指定字符位置开始显示 UTF-8 字符串，支持 ASCII 与完整 GB2312 字符集。
  * @param row 显示行，范围为 1~4
  * @param column 半角字符列，范围为 1~16；中文字符占两列
  * @param text 需要显示的 UTF-8 字符串，可以传入固定字符串或字符数组变量
  */
void My_oled_write_string_at_My(uint8_t row, uint8_t column, const char *text);

/**
  * @brief 在指定位置按 printf 格式显示固定文本或变量数据。
  * @param row 显示行，范围为 1~4
  * @param column 半角字符列，范围为 1~16；中文字符占两列
  * @param format UTF-8 格式字符串；无格式占位符时不需要追加变量参数
  * @param ... 与格式占位符对应的可选变量参数
  */
void My_oled_printf_at_My(uint8_t row, uint8_t column, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* MY_OLED_H */
