#ifndef MY_HWT101_IIC_H
#define MY_HWT101_IIC_H /* 防止 HWT101 软件 IIC 接口头文件被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define MY_HWT101_IIC_ADDRESS       0x50U /* HWT101 的 7 位 IIC 从机地址。 */
#define MY_HWT101_REG_GYRO_X        0x37U /* X 轴角速度数据寄存器地址。 */
#define MY_HWT101_REG_GYRO_Y        0x38U /* Y 轴角速度数据寄存器地址。 */
#define MY_HWT101_REG_GYRO_Z        0x39U /* Z 轴角速度数据寄存器地址。 */
#define MY_HWT101_REG_ROLL          0x3DU /* 横滚角数据寄存器地址。 */
#define MY_HWT101_REG_PITCH         0x3EU /* 俯仰角数据寄存器地址。 */
#define MY_HWT101_REG_YAW           0x3FU /* 偏航角数据寄存器地址。 */

#define MY_HWT101_IIC_OK            0U /* 软件 IIC 操作成功或收到从机应答。 */
#define MY_HWT101_IIC_ERR           1U /* 软件 IIC 操作失败或等待应答超时。 */

typedef struct
{
  int16_t roll_raw;  /* HWT101 返回的横滚角原始有符号值。 */
  int16_t pitch_raw; /* HWT101 返回的俯仰角原始有符号值。 */
  int16_t yaw_raw;   /* HWT101 返回的偏航角原始有符号值。 */
  float roll_deg;    /* 按量程换算后的横滚角，单位为度。 */
  float pitch_deg;   /* 按量程换算后的俯仰角，单位为度。 */
  float yaw_deg;     /* 按量程换算后的偏航角，单位为度。 */
} My_hwt101_angle_t;

typedef struct
{
  int16_t x_raw; /* HWT101 返回的 X 轴角速度原始有符号值。 */
  int16_t y_raw; /* HWT101 返回的 Y 轴角速度原始有符号值。 */
  int16_t z_raw; /* HWT101 返回的 Z 轴角速度原始有符号值。 */
  float x_dps;   /* 按量程换算后的 X 轴角速度，单位为度每秒。 */
  float y_dps;   /* 按量程换算后的 Y 轴角速度，单位为度每秒。 */
  float z_dps;   /* 按量程换算后的 Z 轴角速度，单位为度每秒。 */
} My_hwt101_gyro_t;

/**
  * @brief 初始化 HWT101 软件 IIC 引脚。
  * @details 当前接线沿用旧工程：PC0 为 SCL，PC1 为 SDA。
  *          本函数会自行开启 GPIOC 时钟并配置开漏上拉输出。
  */
void My_hwt101_iic_init_My(void);

/**
  * @brief 产生软件 IIC 起始信号。
  */
void My_hwt101_iic_start_My(void);

/**
  * @brief 产生软件 IIC 停止信号。
  */
void My_hwt101_iic_stop_My(void);

/**
  * @brief 发送一个字节到软件 IIC 总线。
  * @param data 待发送的 8 位数据
  */
void My_hwt101_iic_send_byte_My(uint8_t data);

/**
  * @brief 从软件 IIC 总线读取一个字节。
  * @param ack 读完后是否继续发送 ACK；1 表示 ACK，0 表示 NACK
  * @retval 读取到的 8 位数据
  */
uint8_t My_hwt101_iic_read_byte_My(uint8_t ack);

/**
  * @brief 等待从机 ACK。
  * @retval MY_HWT101_IIC_OK 表示收到 ACK，MY_HWT101_IIC_ERR 表示超时
  */
uint8_t My_hwt101_iic_wait_ack_My(void);

/**
  * @brief 主机发送 ACK。
  */
void My_hwt101_iic_ack_My(void);

/**
  * @brief 主机发送 NACK。
  */
void My_hwt101_iic_nack_My(void);

/**
  * @brief 从 HWT101 连续读取寄存器数据。
  * @param reg 起始寄存器地址
  * @param data 输出缓冲区
  * @param length 读取字节数
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_iic_read_len_My(uint8_t reg, uint8_t *data, uint8_t length);

/**
  * @brief 向 HWT101 连续写入寄存器数据。
  * @param reg 起始寄存器地址
  * @param data 待写入数据缓冲区
  * @param length 写入字节数
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_iic_write_len_My(uint8_t reg, const uint8_t *data, uint8_t length);

/**
  * @brief 检查 HWT101 是否在默认 IIC 地址应答。
  * @retval HAL_OK 表示收到 ACK，HAL_ERROR 表示未收到 ACK
  */
HAL_StatusTypeDef My_hwt101_check_ack_My(void);

/**
  * @brief 读取 HWT101 三轴角度。
  * @param angle 三轴角度输出结构体
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_angle_My(My_hwt101_angle_t *angle);

/**
  * @brief 读取 HWT101 三轴角速度。
  * @param gyro 三轴角速度输出结构体
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_gyro_My(My_hwt101_gyro_t *gyro);

/**
  * @brief 读取 HWT101 偏航角。
  * @param yaw_deg 偏航角输出指针，单位为度
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_yaw_My(float *yaw_deg);

/**
  * @brief 读取 HWT101 Z 轴角速度。
  * @param z_dps Z 轴角速度输出指针，单位为度每秒
  * @retval HAL_OK 表示成功，HAL_ERROR 表示参数错误或通信失败
  */
HAL_StatusTypeDef My_hwt101_read_z_gyro_My(float *z_dps);

#ifdef __cplusplus
}
#endif

#endif /* MY_HWT101_IIC_H */
