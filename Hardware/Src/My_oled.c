#include "My_oled.h"

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>

#define MY_OLED_I2C_DELAY 200U
#define MY_OLED_PIXEL_WIDTH 128U
#define MY_OLED_FORMAT_BUFFER_SIZE 128U
#define MY_OLED_GB2312_FLASH_ADDRESS 0x080C0000UL
#define MY_OLED_GB2312_FONT_MAGIC 0x32334247UL
#define MY_OLED_GB2312_FONT_VERSION 1U
#define MY_OLED_GB2312_GLYPH_COUNT 7445U
#define MY_OLED_GB2312_ENTRY_SIZE 34U
#define MY_OLED_GB2312_HEADER_SIZE 16U

typedef struct
{
  uint32_t magic;      /* 固定字库标识，用于确认 Flash 中已烧录正确数据。 */
  uint16_t version;    /* 字库二进制布局版本。 */
  uint16_t count;      /* 字模条目数量。 */
  uint32_t entry_size; /* 单个字模条目的字节数。 */
  uint32_t reserved;   /* 保留字段，当前固定为零。 */
} My_oled_gb2312_header_t;

typedef struct
{
  uint16_t codepoint; /* Unicode 码点，用于从 UTF-8 文本查找 GB2312 字模。 */
  uint8_t bitmap[32]; /* 16x16 字模，前后各 16 字节对应上下两个 OLED 页。 */
} My_oled_gb2312_glyph_t;

typedef char My_oled_header_size_check_My[
  sizeof(My_oled_gb2312_header_t) == MY_OLED_GB2312_HEADER_SIZE ? 1 : -1];
typedef char My_oled_entry_size_check_My[
  sizeof(My_oled_gb2312_glyph_t) == MY_OLED_GB2312_ENTRY_SIZE ? 1 : -1];

#define MY_OLED_GB2312_HEADER_My \
  ((const My_oled_gb2312_header_t *)(uintptr_t)MY_OLED_GB2312_FLASH_ADDRESS)
#define MY_OLED_GB2312_TABLE_My \
  ((const My_oled_gb2312_glyph_t *)(uintptr_t) \
   (MY_OLED_GB2312_FLASH_ADDRESS + MY_OLED_GB2312_HEADER_SIZE))

#include "My_oled_ascii_font.inc"

/* 软件 I2C 使用 CubeMX 当前配置的 OLED_SCK/OLED_SDA 引脚。 */
#define MY_OLED_SCL_PORT OLED_SCK_GPIO_Port
#define MY_OLED_SCL_PIN  OLED_SCK_Pin
#define MY_OLED_SDA_PORT OLED_SDA_GPIO_Port
#define MY_OLED_SDA_PIN  OLED_SDA_Pin

static uint8_t My_oled_page_My;
static uint8_t My_oled_column_My;
static uint8_t My_oled_address_My;

/* 仅保留常用数字、大写字母和标点，未知字符显示为空格。 */
static const uint8_t My_oled_font_My[39][5] = {
  {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
  {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
  {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
  {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
  {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
  {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
  {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
  {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
  {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
  {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
  {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
  {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
  {0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},
  {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
  {0x00,0x00,0x36,0x00,0x00},
  {0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00}
};

static void My_oled_delay_My(void)
{
  volatile uint32_t ticks = MY_OLED_I2C_DELAY;
  while (ticks > 0U) { ticks--; }
}

static void My_oled_scl_My(uint8_t level)
{
  HAL_GPIO_WritePin(MY_OLED_SCL_PORT, MY_OLED_SCL_PIN,
                    level != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void My_oled_sda_My(uint8_t level)
{
  HAL_GPIO_WritePin(MY_OLED_SDA_PORT, MY_OLED_SDA_PIN,
                    level != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void My_oled_start_My(void)
{
  My_oled_sda_My(1U); My_oled_scl_My(1U); My_oled_delay_My();
  My_oled_sda_My(0U); My_oled_delay_My(); My_oled_scl_My(0U);
}

static void My_oled_stop_My(void)
{
  My_oled_sda_My(0U); My_oled_delay_My(); My_oled_scl_My(1U);
  My_oled_delay_My(); My_oled_sda_My(1U); My_oled_delay_My();
}

/**
  * @brief 通过软件 I2C 发送一个字节并读取从机应答。
  * @param value 待发送字节
  * @retval HAL_OK 表示收到 ACK，HAL_ERROR 表示收到 NACK
  */
static HAL_StatusTypeDef My_oled_send_byte_My(uint8_t value)
{
  uint8_t bit_index;
  GPIO_PinState ack_state;

  for (bit_index = 0U; bit_index < 8U; bit_index++)
  {
    My_oled_sda_My((uint8_t)((value & 0x80U) != 0U));
    value <<= 1U;
    My_oled_delay_My(); My_oled_scl_My(1U); My_oled_delay_My();
    My_oled_scl_My(0U); My_oled_delay_My();
  }
  /* 释放 SDA，给从机 ACK 一个时钟。 */
  My_oled_sda_My(1U); My_oled_delay_My();
  My_oled_scl_My(1U); My_oled_delay_My();
  ack_state = HAL_GPIO_ReadPin(MY_OLED_SDA_PORT, MY_OLED_SDA_PIN);
  My_oled_scl_My(0U); My_oled_delay_My();

  return ack_state == GPIO_PIN_RESET ? HAL_OK : HAL_ERROR;
}

/**
  * @brief 向当前 OLED 地址写入命令或显示数据。
  * @param control 控制字节，0x00 表示命令，0x40 表示显示数据
  * @param data 待发送数据
  * @param length 数据长度
  * @retval HAL_OK 表示全部字节收到 ACK，否则返回 HAL_ERROR
  */
static HAL_StatusTypeDef My_oled_write_bytes_My(uint8_t control,
                                                 const uint8_t *data,
                                                 uint16_t length)
{
  uint16_t index;

  if (My_oled_address_My == 0U || data == NULL)
  {
    return HAL_ERROR;
  }

  My_oled_start_My();
  if (My_oled_send_byte_My((uint8_t)(My_oled_address_My << 1U)) != HAL_OK ||
      My_oled_send_byte_My(control) != HAL_OK)
  {
    My_oled_stop_My();
    return HAL_ERROR;
  }

  for (index = 0U; index < length; index++)
  {
    if (My_oled_send_byte_My(data[index]) != HAL_OK)
    {
      My_oled_stop_My();
      return HAL_ERROR;
    }
  }
  My_oled_stop_My();
  return HAL_OK;
}

/**
  * @brief 向 OLED 发送单条命令。
  * @param command SSD1306 命令
  * @retval HAL 状态
  */
static HAL_StatusTypeDef My_oled_command_My(uint8_t command)
{
  return My_oled_write_bytes_My(0x00U, &command, 1U);
}

/**
  * @brief 发送地址字节，探测指定的 OLED 地址。
  * @param address 7 位 I2C 地址
  * @retval HAL_OK 表示收到 ACK，否则返回 HAL_ERROR
  */
static HAL_StatusTypeDef My_oled_probe_address_My(uint8_t address)
{
  HAL_StatusTypeDef status;

  My_oled_start_My();
  status = My_oled_send_byte_My((uint8_t)(address << 1U));
  My_oled_stop_My();
  return status;
}

/**
  * @brief 释放可能被从机占用的软件 I2C 总线。
  */
static void My_oled_recover_bus_My(void)
{
  uint8_t clock_index;

  My_oled_sda_My(1U);
  for (clock_index = 0U; clock_index < 9U; clock_index++)
  {
    My_oled_scl_My(0U); My_oled_delay_My();
    My_oled_scl_My(1U); My_oled_delay_My();
  }
  My_oled_stop_My();
}

/**
  * @brief 初始化 OLED，并自动探测常见地址。
  * @retval HAL_OK 表示初始化完成，否则返回 HAL_ERROR
  */
HAL_StatusTypeDef My_oled_init_My(void)
{
  GPIO_InitTypeDef gpio_init = {0};
  static const uint8_t init_commands[] = {
    0xAEU,0xD5U,0x80U,0xA8U,0x3FU,0xD3U,0x00U,0x40U,
    0x8DU,0x14U,0x20U,0x02U,0xA1U,0xC8U,0xDAU,0x12U,
    0x81U,0x7FU,0xD9U,0xF1U,0xDBU,0x40U,0xA4U,0xA6U,0xAFU
  };
  uint16_t command_index;

  __HAL_RCC_GPIOF_CLK_ENABLE();
  gpio_init.Pin = MY_OLED_SCL_PIN | MY_OLED_SDA_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MY_OLED_SCL_PORT, &gpio_init);
  My_oled_scl_My(1U);
  My_oled_sda_My(1U);
  HAL_Delay(100U);

  My_oled_address_My = 0U;
  My_oled_recover_bus_My();
  if (My_oled_probe_address_My(MY_OLED_I2C_ADDRESS_LOW) == HAL_OK)
  {
    My_oled_address_My = MY_OLED_I2C_ADDRESS_LOW;
  }
  else if (My_oled_probe_address_My(MY_OLED_I2C_ADDRESS_HIGH) == HAL_OK)
  {
    My_oled_address_My = MY_OLED_I2C_ADDRESS_HIGH;
  }
  else
  {
    return HAL_ERROR;
  }

  for (command_index = 0U; command_index < sizeof(init_commands); command_index++)
  {
    if (My_oled_command_My(init_commands[command_index]) != HAL_OK)
    {
      My_oled_address_My = 0U;
      return HAL_ERROR;
    }
  }
  My_oled_clear_My();
  return HAL_OK;
}

/**
  * @brief 获取初始化时探测到的 OLED 地址。
  * @retval OLED 的 7 位 I2C 地址，未探测到时返回 0
  */
uint8_t My_oled_get_address_My(void)
{
  return My_oled_address_My;
}

void My_oled_set_cursor_My(uint8_t page, uint8_t column)
{
  My_oled_page_My = page & 0x07U; My_oled_column_My = column & 0x7FU;
  My_oled_command_My((uint8_t)(0xB0U | My_oled_page_My));
  My_oled_command_My((uint8_t)(0x00U | (My_oled_column_My & 0x0FU)));
  My_oled_command_My((uint8_t)(0x10U | (My_oled_column_My >> 4U)));
}

void My_oled_clear_My(void)
{
  uint8_t blank[128]; uint8_t page; uint16_t index;
  for (index = 0U; index < sizeof(blank); index++) { blank[index] = 0U; }
  for (page = 0U; page < 8U; page++) { My_oled_set_cursor_My(page, 0U); My_oled_write_bytes_My(0x40U, blank, sizeof(blank)); }
  My_oled_set_cursor_My(0U, 0U);
}

void My_oled_write_char_My(char value)
{
  uint8_t glyph[6] = {0U,0U,0U,0U,0U,0U};
  uint8_t index;
  if (value >= '0' && value <= '9') { for (index = 0U; index < 5U; index++) { glyph[index] = My_oled_font_My[value - '0'][index]; } }
  else if (value >= 'A' && value <= 'Z') { for (index = 0U; index < 5U; index++) { glyph[index] = My_oled_font_My[10U + value - 'A'][index]; } }
  else if (value == ':') { for (index = 0U; index < 5U; index++) { glyph[index] = My_oled_font_My[36U][index]; } }
  else if (value == '-') { for (index = 0U; index < 5U; index++) { glyph[index] = My_oled_font_My[37U][index]; } }
  else if (value == '.') { for (index = 0U; index < 5U; index++) { glyph[index] = My_oled_font_My[38U][index]; } }
  My_oled_write_bytes_My(0x40U, glyph, sizeof(glyph));
  My_oled_column_My = (uint8_t)(My_oled_column_My + 6U);
  if (My_oled_column_My > 122U) { My_oled_column_My = 0U; My_oled_page_My = (uint8_t)((My_oled_page_My + 1U) & 0x07U); My_oled_set_cursor_My(My_oled_page_My, 0U); }
}

void My_oled_write_string_My(const char *text)
{
  if (text == NULL) { return; }
  while (*text != '\0') { My_oled_write_char_My(*text); text++; }
}

/**
  * @brief 从 UTF-8 字节流解码一个 Unicode 码点。
  * @param text 当前 UTF-8 字符的首地址
  * @param codepoint 解码后的 Unicode 码点输出地址
  * @retval 本次消耗的字节数；非法序列按一个字节处理并输出问号
  */
static uint8_t My_oled_decode_utf8_My(const uint8_t *text, uint32_t *codepoint)
{
  if (text[0] < 0x80U)
  {
    *codepoint = text[0];
    return 1U;
  }
  if ((text[0] & 0xE0U) == 0xC0U && text[1] != '\0' &&
      (text[1] & 0xC0U) == 0x80U)
  {
    *codepoint = ((uint32_t)(text[0] & 0x1FU) << 6U) |
                 (uint32_t)(text[1] & 0x3FU);
    return 2U;
  }
  if ((text[0] & 0xF0U) == 0xE0U && text[1] != '\0' && text[2] != '\0' &&
      (text[1] & 0xC0U) == 0x80U && (text[2] & 0xC0U) == 0x80U)
  {
    *codepoint = ((uint32_t)(text[0] & 0x0FU) << 12U) |
                 ((uint32_t)(text[1] & 0x3FU) << 6U) |
                 (uint32_t)(text[2] & 0x3FU);
    return 3U;
  }
  if ((text[0] & 0xF8U) == 0xF0U && text[1] != '\0' &&
      text[2] != '\0' && text[3] != '\0' &&
      (text[1] & 0xC0U) == 0x80U && (text[2] & 0xC0U) == 0x80U &&
      (text[3] & 0xC0U) == 0x80U)
  {
    *codepoint = ((uint32_t)(text[0] & 0x07U) << 18U) |
                 ((uint32_t)(text[1] & 0x3FU) << 12U) |
                 ((uint32_t)(text[2] & 0x3FU) << 6U) |
                 (uint32_t)(text[3] & 0x3FU);
    return 4U;
  }

  *codepoint = (uint32_t)'?';
  return 1U;
}

/**
  * @brief 使用二分查找获取 GB2312 字符的 16x16 字模。
  * @param codepoint 需要查找的 Unicode 码点
  * @retval 找到时返回字模地址，不属于 GB2312 字符集时返回空指针
  */
static const uint8_t *My_oled_find_gb2312_glyph_My(uint32_t codepoint)
{
  uint16_t left = 0U;                          /* 当前二分查找区间左边界。 */
  uint16_t right = MY_OLED_GB2312_GLYPH_COUNT; /* 当前二分查找区间右边界。 */

  if (codepoint > 0xFFFFU ||
      MY_OLED_GB2312_HEADER_My->magic != MY_OLED_GB2312_FONT_MAGIC ||
      MY_OLED_GB2312_HEADER_My->version != MY_OLED_GB2312_FONT_VERSION ||
      MY_OLED_GB2312_HEADER_My->count != MY_OLED_GB2312_GLYPH_COUNT ||
      MY_OLED_GB2312_HEADER_My->entry_size != MY_OLED_GB2312_ENTRY_SIZE)
  {
    return NULL;
  }

  while (left < right)
  {
    uint16_t middle = (uint16_t)(left + (uint16_t)((right - left) / 2U)); /* 区间中点。 */
    uint16_t middle_codepoint = MY_OLED_GB2312_TABLE_My[middle].codepoint; /* 中点码点。 */

    if ((uint16_t)codepoint == middle_codepoint)
    {
      return MY_OLED_GB2312_TABLE_My[middle].bitmap;
    }
    if ((uint16_t)codepoint < middle_codepoint)
    {
      right = middle;
    }
    else
    {
      left = (uint16_t)(middle + 1U);
    }
  }

  return NULL;
}

/**
  * @brief 在指定 OLED 页和像素列写入一个 16 像素高的字模。
  * @param page 字模上半部分所在页，范围为 0、2、4、6
  * @param pixel_column 起始像素列
  * @param bitmap 上下两页连续存放的字模数据
  * @param width 字模宽度，ASCII 为 8，中文为 16
  * @retval HAL 状态
  */
static HAL_StatusTypeDef My_oled_write_glyph_16_My(uint8_t page,
                                                    uint8_t pixel_column,
                                                    const uint8_t *bitmap,
                                                    uint8_t width)
{
  My_oled_set_cursor_My(page, pixel_column);
  if (My_oled_write_bytes_My(0x40U, bitmap, width) != HAL_OK)
  {
    return HAL_ERROR;
  }

  My_oled_set_cursor_My((uint8_t)(page + 1U), pixel_column);
  return My_oled_write_bytes_My(0x40U, &bitmap[width], width);
}

/**
  * @brief 从指定字符位置开始显示 UTF-8 字符串，支持 ASCII 与完整 GB2312 字符集。
  * @param row 显示行，范围为 1~4
  * @param column 半角字符列，范围为 1~16；中文字符占两列
  * @param text 需要显示的 UTF-8 字符串
  */
void My_oled_write_string_at_My(uint8_t row, uint8_t column, const char *text)
{
  const uint8_t *cursor = (const uint8_t *)text; /* 当前待解析的 UTF-8 字节位置。 */
  uint8_t page;                                  /* 当前文本行对应的 OLED 起始页。 */
  uint8_t pixel_column;                          /* 当前字符的起始像素列。 */

  if (row == 0U || row > MY_OLED_ROW_COUNT ||
      column == 0U || column > MY_OLED_CHARACTER_COLUMN_COUNT ||
      text == NULL)
  {
    return;
  }

  page = (uint8_t)((row - 1U) * 2U);
  pixel_column = (uint8_t)((column - 1U) * MY_OLED_CHARACTER_WIDTH);
  while (*cursor != '\0')
  {
    const uint8_t *bitmap; /* 当前字符需要写入 OLED 的字模地址。 */
    uint32_t codepoint;    /* 当前 UTF-8 字符对应的 Unicode 码点。 */
    uint8_t consumed;      /* 当前 UTF-8 字符消耗的字节数。 */
    uint8_t width;         /* 当前字符占用的像素宽度。 */

    consumed = My_oled_decode_utf8_My(cursor, &codepoint);
    if (codepoint >= 0x20U && codepoint <= 0x7EU)
    {
      bitmap = My_oled_ascii_8x16_My[codepoint - 0x20U];
      width = 8U;
    }
    else
    {
      bitmap = My_oled_find_gb2312_glyph_My(codepoint);
      width = 16U;
      if (bitmap == NULL)
      {
        bitmap = My_oled_ascii_8x16_My['?' - 0x20U];
        width = 8U;
      }
    }

    if ((uint16_t)pixel_column + width > MY_OLED_PIXEL_WIDTH)
    {
      break;
    }
    if (My_oled_write_glyph_16_My(page, pixel_column, bitmap, width) != HAL_OK)
    {
      break;
    }

    pixel_column = (uint8_t)(pixel_column + width);
    cursor += consumed;
  }
}

/**
  * @brief 在指定位置按 printf 格式显示固定文本或变量数据。
  * @param row 显示行，范围为 1~4
  * @param column 半角字符列，范围为 1~16；中文字符占两列
  * @param format UTF-8 格式字符串；无格式占位符时不需要追加变量参数
  * @param ... 与格式占位符对应的可选变量参数
  */
void My_oled_printf_at_My(uint8_t row, uint8_t column, const char *format, ...)
{
  char buffer[MY_OLED_FORMAT_BUFFER_SIZE]; /* 保存格式化后的 UTF-8 显示字符串。 */
  va_list arguments; /* 可变参数读取状态。 */
  int format_result; /* 格式化结果，负数表示格式化失败。 */

  if (format == NULL)
  {
    return;
  }

  va_start(arguments, format);
  format_result = vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);
  if (format_result < 0)
  {
    return;
  }

  buffer[sizeof(buffer) - 1U] = '\0';
  My_oled_write_string_at_My(row, column, buffer);
}
