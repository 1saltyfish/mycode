#ifndef __SGP30_H
#define __SGP30_H

#include "zf_common_headfile.h"

#define  SGP30_SCL_SET           gpio_set_level(B15, 1)
#define  SGP30_SCL_RESET         gpio_set_level(B15, 0)
#define  SGP30_SDA_SET           gpio_set_level(B14, 1)
#define  SGP30_SDA_RESET         gpio_set_level(B14, 0)

// GPIO 引脚宏定义
#define  SGP30_SDA_READ()        gpio_get_level(B14)


#define SGP30_read  0xb1  //SGP30的读地址
#define SGP30_write 0xb0  //SGP30的写地址


void SGP30_IIC_Start(void);				
void SGP30_IIC_Stop(void);	  		
void SGP30_IIC_Send_Byte(uint8 txd);	
unsigned int SGP30_IIC_Read_Byte(unsigned char ack); 
unsigned char SGP30_IIC_Wait_Ack(void); 			
void SGP30_IIC_Ack(void);				
void SGP30_IIC_NAck(void);				
void SGP30_IIC_Write_One_Byte(uint8 daddr,uint8 addr,uint8 data);
unsigned char SGP30_IIC_Read_One_Byte(uint8 daddr,uint8 addr);	
void SGP30_Init(void);				  
void SGP30_Write(uint8 a, uint8 b);
uint32 SGP30_Read(void);


#endif
