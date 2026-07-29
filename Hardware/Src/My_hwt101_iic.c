#include "My_hwt101_iic.h"
#include <stddef.h>

#ifndef HWT101_SCL_Pin
#define HWT101_SCL_Pin                   GPIO_PIN_0
#endif

#ifndef HWT101_SCL_GPIO_Port
#define HWT101_SCL_GPIO_Port             GPIOC
#endif

#ifndef HWT101_SDA_Pin
#define HWT101_SDA_Pin                   GPIO_PIN_1
#endif

#ifndef HWT101_SDA_GPIO_Port
#define HWT101_SDA_GPIO_Port             GPIOC
#endif

#define MY_HWT101_IIC_SCL_PORT        HWT101_SCL_GPIO_Port
#define MY_HWT101_IIC_SDA_PORT        HWT101_SDA_GPIO_Port
#define MY_HWT101_IIC_SCL_PIN         HWT101_SCL_Pin
#define MY_HWT101_IIC_SDA_PIN         HWT101_SDA_Pin
#define MY_HWT101_IIC_DELAY_UNIT      160U
#define MY_HWT101_IIC_ACK_TIMEOUT     250U
#define MY_HWT101_ANGLE_SCALE         (180.0f / 32768.0f)
#define MY_HWT101_GYRO_SCALE          (2000.0f / 32768.0f)

/**
  * @brief 软件 IIC 短延时。
  * @param count 延时循环次数倍率
  */
static void My_hwt101_iic_delay_My(uint32_t count)
{
  volatile uint32_t ticks = count * MY_HWT101_IIC_DELAY_UNIT;

  while (ticks > 0U)
  {
    ticks--;
  }
}

/**
  * @brief 配置 SDA 为输入，用于释放总线或读取从机数据。
  */
static void My_hwt101_sda_input_My(void)
{
  MY_HWT101_IIC_SDA_PORT->MODER &= ~(3UL << (1U * 2U));
}

/**
  * @brief 配置 SDA 为开漏输出，用于主机拉低或释放 SDA。
  */
static void My_hwt101_sda_output_My(void)
{
  MY_HWT101_IIC_SDA_PORT->MODER &= ~(3UL << (1U * 2U));
  MY_HWT101_IIC_SDA_PORT->MODER |= (1UL << (1U * 2U));
}

/**
  * @brief 设置 SCL 电平。
  * @param level 0 表示低电平，非 0 表示高电平
  */
static void My_hwt101_set_scl_My(uint8_t level)
{
  if (level != 0U)
  {
    MY_HWT101_IIC_SCL_PORT->BSRR = MY_HWT101_IIC_SCL_PIN;
  }
  else
  {
    MY_HWT101_IIC_SCL_PORT->BSRR = ((uint32_t)MY_HWT101_IIC_SCL_PIN << 16U);
  }
}

/**
  * @brief 设置 SDA 电平。
  * @param level 0 表示低电平，非 0 表示高电平
  */
static void My_hwt101_set_sda_My(uint8_t level)
{
  if (level != 0U)
  {
    MY_HWT101_IIC_SDA_PORT->BSRR = MY_HWT101_IIC_SDA_PIN;
  }
  else
  {
    MY_HWT101_IIC_SDA_PORT->BSRR = ((uint32_t)MY_HWT101_IIC_SDA_PIN << 16U);
  }
}

/**
  * @brief 读取 SDA 当前电平。
  * @retval 0 表示低电平，1 表示高电平
  */
static uint8_t My_hwt101_read_sda_My(void)
{
  return (uint8_t)((MY_HWT101_IIC_SDA_PORT->IDR & MY_HWT101_IIC_SDA_PIN) != 0U);
}

/**
  * @brief 将两个低字节在前的数据合成为有符号 16 位数。
  * @param low 低字节
  * @param high 高字节
  * @retval 合成后的有符号原始值
  */
static int16_t My_hwt101_make_int16_My(uint8_t low, uint8_t high)
{
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

/**
  * @brief 初始化 HWT101 软件 IIC 引脚。
  * @details 当前接线沿用旧工程：PC0 为 SCL，PC1 为 SDA。
  *          这里不修改 CubeMX 生成的 GPIO 初始化函数，避免下次生成代码被覆盖。
  */
void My_hwt101_iic_init_My(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio_init.Pin = MY_HWT101_IIC_SCL_PIN | MY_HWT101_IIC_SDA_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MY_HWT101_IIC_SCL_PORT, &gpio_init);

  My_hwt101_set_scl_My(1U);
  My_hwt101_set_sda_My(1U);
  My_hwt101_iic_delay_My(2U);
}

/**
  * @brief 产生软件 IIC 起始信号。
  */
void My_hwt101_iic_start_My(void)
{
  My_hwt101_sda_output_My();
  My_hwt101_set_sda_My(1U);
  My_hwt101_set_scl_My(1U);
  My_hwt101_iic_delay_My(2U);
  My_hwt101_set_sda_My(0U);
  My_hwt101_iic_delay_My(2U);
  My_hwt101_set_scl_My(0U);
}

/**
  * @brief 产生软件 IIC 停止信号。
  */
void My_hwt101_iic_stop_My(void)
{
  My_hwt101_sda_output_My();
  My_hwt101_set_scl_My(0U);
  My_hwt101_set_sda_My(0U);
  My_hwt101_iic_delay_My(1U);
  My_hwt101_set_scl_My(1U);
  My_hwt101_iic_delay_My(1U);
  My_hwt101_set_sda_My(1U);
  My_hwt101_iic_delay_My(1U);
}

/**
  * @brief 等待从机 ACK。
  * @retval MY_HWT101_IIC_OK 表示收到 ACK，MY_HWT101_IIC_ERR 表示超时
  */
uint8_t My_hwt101_iic_wait_ack_My(void)
{
  uint8_t timeout = 0U;

  My_hwt101_sda_input_My();
  My_hwt101_set_sda_My(1U);
  My_hwt101_iic_delay_My(1U);
  My_hwt101_set_scl_My(1U);
  My_hwt101_iic_delay_My(1U);

  while (My_hwt101_read_sda_My() != 0U)
  {
    timeout++;
    if (timeout > MY_HWT101_IIC_ACK_TIMEOUT)
    {
      My_hwt101_iic_stop_My();
      return MY_HWT101_IIC_ERR;
    }
  }

  My_hwt101_set_scl_My(0U);
  return MY_HWT101_IIC_OK;
}

/**
  * @brief 主机发送 ACK。
  */
void My_hwt101_iic_ack_My(void)
{
  My_hwt101_set_scl_My(0U);
  My_hwt101_sda_output_My();
  My_hwt101_set_sda_My(0U);
  My_hwt101_iic_delay_My(1U);
  My_hwt101_set_scl_My(1U);
  My_hwt101_iic_delay_My(1U);
  My_hwt101_set_scl_My(0U);
}

/**
  * @brief 主机发送 NACK。
  */
void My_hwt101_iic_nack_My(void)
{
  My_hwt101_set_scl_My(0U);
  My_hwt101_sda_output_My();
  My_hwt101_set_sda_My(1U);
  My_hwt101_iic_delay_My(1U);
  My_hwt101_set_scl_My(1U);
  My_hwt101_iic_delay_My(1U);
  My_hwt101_set_scl_My(0U);
}

/**
  * @brief 发送一个字节到软件 IIC 总线。
  * @param data 待发送的 8 位数据
  */
void My_hwt101_iic_send_byte_My(uint8_t data)
{
  uint8_t bit_index;

  My_hwt101_sda_output_My();
  My_hwt101_set_scl_My(0U);

  for (bit_index = 0U; bit_index < 8U; bit_index++)
  {
    My_hwt101_set_sda_My((uint8_t)((data & 0x80U) != 0U));
    data <<= 1U;
    My_hwt101_iic_delay_My(1U);
    My_hwt101_set_scl_My(1U);
    My_hwt101_iic_delay_My(1U);
    My_hwt101_set_scl_My(0U);
    My_hwt101_iic_delay_My(1U);
  }
}

/**
  * @brief 从软件 IIC 总线读取一个字节。
  * @param ack 读完后是否继续发送 ACK；1 表示 ACK，0 表示 NACK
  * @retval 读取到的 8 位数据
  */
uint8_t My_hwt101_iic_read_byte_My(uint8_t ack)
{
  uint8_t bit_index;
  uint8_t data = 0U;

  My_hwt101_set_sda_My(1U);
  My_hwt101_sda_input_My();

  for (bit_index = 0U; bit_index < 8U; bit_index++)
  {
    My_hwt101_set_scl_My(0U);
    My_hwt101_iic_delay_My(1U);
    My_hwt101_set_scl_My(1U);
    data <<= 1U;
    if (My_hwt101_read_sda_My() != 0U)
    {
      data++;
    }
    My_hwt101_iic_delay_My(1U);
  }

  if (ack == 0U)
  {
    My_hwt101_iic_nack_My();
  }
  else
  {
    My_hwt101_iic_ack_My();
  }

  return data;
}

/**
  * @brief 从 HWT101 连续读取寄存器数据。
  * @param reg 起始寄存器地址
  * @param data 输出缓冲区
  * @param length 读取字节数
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_iic_read_len_My(uint8_t reg, uint8_t *data, uint8_t length)
{
  if (data == NULL || length == 0U)
  {
    return HAL_ERROR;
  }

  My_hwt101_iic_start_My();
  My_hwt101_iic_send_byte_My((uint8_t)(MY_HWT101_IIC_ADDRESS << 1U));
  if (My_hwt101_iic_wait_ack_My() != MY_HWT101_IIC_OK)
  {
    return HAL_ERROR;
  }

  My_hwt101_iic_send_byte_My(reg);
  if (My_hwt101_iic_wait_ack_My() != MY_HWT101_IIC_OK)
  {
    return HAL_ERROR;
  }

  My_hwt101_iic_start_My();
  My_hwt101_iic_send_byte_My((uint8_t)((MY_HWT101_IIC_ADDRESS << 1U) | 0x01U));
  if (My_hwt101_iic_wait_ack_My() != MY_HWT101_IIC_OK)
  {
    return HAL_ERROR;
  }

  while (length > 0U)
  {
    *data = My_hwt101_iic_read_byte_My((uint8_t)(length > 1U));
    data++;
    length--;
  }

  My_hwt101_iic_stop_My();
  return HAL_OK;
}

/**
  * @brief 向 HWT101 连续写入寄存器数据。
  * @param reg 起始寄存器地址
  * @param data 待写入数据缓冲区
  * @param length 写入字节数
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_iic_write_len_My(uint8_t reg, const uint8_t *data, uint8_t length)
{
  uint8_t index;

  if (data == NULL || length == 0U)
  {
    return HAL_ERROR;
  }

  My_hwt101_iic_start_My();
  My_hwt101_iic_send_byte_My((uint8_t)(MY_HWT101_IIC_ADDRESS << 1U));
  if (My_hwt101_iic_wait_ack_My() != MY_HWT101_IIC_OK)
  {
    return HAL_ERROR;
  }

  My_hwt101_iic_send_byte_My(reg);
  if (My_hwt101_iic_wait_ack_My() != MY_HWT101_IIC_OK)
  {
    return HAL_ERROR;
  }

  for (index = 0U; index < length; index++)
  {
    My_hwt101_iic_send_byte_My(data[index]);
    if (My_hwt101_iic_wait_ack_My() != MY_HWT101_IIC_OK)
    {
      return HAL_ERROR;
    }
  }

  My_hwt101_iic_stop_My();
  return HAL_OK;
}

/**
  * @brief 检查 HWT101 是否在默认 IIC 地址应答。
  * @retval HAL_OK 表示收到 ACK，HAL_ERROR 表示未收到 ACK
  */
HAL_StatusTypeDef My_hwt101_check_ack_My(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  My_hwt101_iic_start_My();
  My_hwt101_iic_send_byte_My((uint8_t)(MY_HWT101_IIC_ADDRESS << 1U));
  if (My_hwt101_iic_wait_ack_My() != MY_HWT101_IIC_OK)
  {
    status = HAL_ERROR;
  }
  My_hwt101_iic_stop_My();

  return status;
}

/**
  * @brief 读取 HWT101 三轴角度。
  * @param angle 三轴角度输出结构体
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_angle_My(My_hwt101_angle_t *angle)
{
  uint8_t buffer[6];

  if (angle == NULL)
  {
    return HAL_ERROR;
  }

  if (My_hwt101_iic_read_len_My(MY_HWT101_REG_ROLL, buffer, sizeof(buffer)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  angle->roll_raw = My_hwt101_make_int16_My(buffer[0], buffer[1]);
  angle->pitch_raw = My_hwt101_make_int16_My(buffer[2], buffer[3]);
  angle->yaw_raw = My_hwt101_make_int16_My(buffer[4], buffer[5]);
  angle->roll_deg = (float)angle->roll_raw * MY_HWT101_ANGLE_SCALE;
  angle->pitch_deg = (float)angle->pitch_raw * MY_HWT101_ANGLE_SCALE;
  angle->yaw_deg = (float)angle->yaw_raw * MY_HWT101_ANGLE_SCALE;

  return HAL_OK;
}

/**
  * @brief 读取 HWT101 三轴角速度。
  * @param gyro 三轴角速度输出结构体
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_gyro_My(My_hwt101_gyro_t *gyro)
{
  uint8_t buffer[6];

  if (gyro == NULL)
  {
    return HAL_ERROR;
  }

  if (My_hwt101_iic_read_len_My(MY_HWT101_REG_GYRO_X, buffer, sizeof(buffer)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  gyro->x_raw = My_hwt101_make_int16_My(buffer[0], buffer[1]);
  gyro->y_raw = My_hwt101_make_int16_My(buffer[2], buffer[3]);
  gyro->z_raw = My_hwt101_make_int16_My(buffer[4], buffer[5]);
  gyro->x_dps = (float)gyro->x_raw * MY_HWT101_GYRO_SCALE;
  gyro->y_dps = (float)gyro->y_raw * MY_HWT101_GYRO_SCALE;
  gyro->z_dps = (float)gyro->z_raw * MY_HWT101_GYRO_SCALE;

  return HAL_OK;
}

/**
  * @brief 读取 HWT101 偏航角。
  * @param yaw_deg 偏航角输出指针，单位为度
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_yaw_My(float *yaw_deg)
{
  My_hwt101_angle_t angle;

  if (yaw_deg == NULL)
  {
    return HAL_ERROR;
  }

  if (My_hwt101_read_angle_My(&angle) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *yaw_deg = angle.yaw_deg;
  return HAL_OK;
}

/**
  * @brief 读取 HWT101 Z 轴角速度。
  * @param z_dps Z 轴角速度输出指针，单位为度每秒
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_z_gyro_My(float *z_dps)
{
  My_hwt101_gyro_t gyro;

  if (z_dps == NULL)
  {
    return HAL_ERROR;
  }

  if (My_hwt101_read_gyro_My(&gyro) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *z_dps = gyro.z_dps;
  return HAL_OK;
}
