#ifndef __HWT101_H__
#define __HWT101_H__

#include "main.h"
#include "IIC.h"

extern volatile float Angle;

void HWT101_getAngle(void);
void HWT101_getAngle_Reset(void);

#endif
