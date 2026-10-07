#include "HWT101.h"

uint8_t Angle_I2C[2];
volatile float Angle;
int yaw = 0;
int yaw_first = 0;
float Angle_first = 0;

/*
 * @brief  读取HWT101角度
 * @param  无
 * @retval 无
 */
void HWT101_getAngle(void)
{
	IIC_Read_Len(0x50, 0x3F, 2, Angle_I2C);
	yaw = ((uint16_t)Angle_I2C[1] << 8 | Angle_I2C[0]);
	Angle = (float)(yaw) / 32768.0f * 180 - Angle_first + 360;
	if (Angle >= 360)
		Angle -= 360;
	Angle -= 360;	// 这两行目的是使陀螺仪角度Angle值顺时针0~360
	Angle = -Angle; //
}

/*
 * @brief  读取HWT101角度并重置偏移量
 * @param  无
 * @retval 无
 */
void HWT101_getAngle_Reset(void)
{
	IIC_Init();
	HAL_Delay(100);
	IIC_Read_Len(0x50, 0x3F, 2, Angle_I2C);
	yaw_first = (((uint16_t)Angle_I2C[1] << 8 | Angle_I2C[0]));
	Angle_first = (float)(yaw_first) / 32768.0f * 180;
}
