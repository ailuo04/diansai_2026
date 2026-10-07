#ifndef __IIC_H__
#define __IIC_H__

#include "main.h"

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define BITBAND(addr, bitnum) (((addr) & 0xF0000000U) + 0x02000000U + (((addr) & 0x000FFFFFU) << 5U) + ((bitnum) << 2U))
#define MEM_ADDR(addr) (*((volatile unsigned long *)(addr)))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND((addr), (bitnum)))

#define GPIOC_ODR_Addr (GPIOC_BASE + 20U)
#define GPIOC_IDR_Addr (GPIOC_BASE + 16U)
#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, (n))
#define PCin(n)  BIT_ADDR(GPIOC_IDR_Addr, (n))

#define SDA_IN()  do { GPIOC->MODER &= ~(3UL << (1U * 2U)); } while (0)
#define SDA_OUT() do { GPIOC->MODER &= ~(3UL << (1U * 2U)); GPIOC->MODER |= 1UL << (1U * 2U); } while (0)

#define IIC_SCL  PCout(0U)
#define IIC_SDA  PCout(1U)
#define READ_SDA PCin(1U)

void delayiic(u32 m);
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
u8 IIC_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf);
u8 IIC_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf);

#endif
