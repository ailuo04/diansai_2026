#include "My_oled.h"

#include <stddef.h>

#define MY_OLED_I2C_DELAY 200U

/* 软件 I2C 使用 CubeMX 当前配置的 OLED_SCK/OLED_SDA 引脚。 */
#define MY_OLED_SCL_PORT OLED_SCK_GPIO_Port
#define MY_OLED_SCL_PIN  OLED_SCK_Pin
#define MY_OLED_SDA_PORT OLED_SDA_GPIO_Port
#define MY_OLED_SDA_PIN  OLED_SDA_Pin

static uint8_t My_oled_page_My;
static uint8_t My_oled_column_My;
static uint8_t My_oled_address_My;

/* 仅保留常用数字、大写字母和标点，未知字符显示为空格。 */
static const uint8_t My_oled_font_My[37][5] = {
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
  {0x00,0x00,0x36,0x00,0x00}
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
  My_oled_write_bytes_My(0x40U, glyph, sizeof(glyph));
  My_oled_column_My = (uint8_t)(My_oled_column_My + 6U);
  if (My_oled_column_My > 122U) { My_oled_column_My = 0U; My_oled_page_My = (uint8_t)((My_oled_page_My + 1U) & 0x07U); My_oled_set_cursor_My(My_oled_page_My, 0U); }
}

void My_oled_write_string_My(const char *text)
{
  if (text == NULL) { return; }
  while (*text != '\0') { My_oled_write_char_My(*text); text++; }
}
