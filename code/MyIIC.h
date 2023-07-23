#ifndef __MyIIC_H
#define __MyIIC_H

#include "zf_common_headfile.h"

#define	I2C_SCL_SET()  gpio_set_level(B12, 1)
#define I2C_SCL_CLR()  gpio_set_level(B12, 0)
#define I2C_SDA_SET()  gpio_set_level(E4, 1)
#define I2C_SDA_CLR()  gpio_set_level(E4, 0)
#define I2C_SDA_Read() gpio_get_level(E4)

#define I2C_Ack    0
#define I2C_NO_Ack 1

void My_IIC_GPIO_Init(void);
void My_I2C_Start(void);
void My_I2C_Stop(void);
unsigned char My_I2C_Wait_Ack(void);
void My_I2C_Ack(void);
void My_I2C_Nack();
void My_I2C_WriteByte(unsigned char I2C_Byte);
unsigned char My_I2C_ReadByte(uint8 ack);

#endif
