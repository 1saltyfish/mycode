#include "AP3216.h"
#include "MyIIC.h"
#include "zf_common_headfile.h"

uint8 ap3216c_init(void)
{
	uint8 temp = 0;
	
	My_IIC_GPIO_Init();
	ap3216c_write_one_byte(0x00, 0x04);  /*复位AP3216C */
	system_delay_ms (50);                /* AP3216C复位至少10ms */
	ap3216c_write_one_byte(0x00, 0x03);  /* 开启ALS、PS+IR */
	
	temp = ap3216c_read_one_byte(0x00);  /* 读取刚刚写进去的0x03 */
	if(temp == 0x03)
	{
		return 0;    /* AP3216C正常 */
	}
	else
	{
		return 1;    /* AP33216C失败 */
	}
}

void ap3216c_read_data(uint16 *ir, uint16 *ps, uint16_t *als)
{
	uint8 buf[6];
	uint8 i;
	
	for(i = 0; i < 6; i++)
	{
		buf[i] = ap3216c_read_one_byte(0X0A + i);                   /* 循环读取所有传感器数据 */
	}
	
	if(buf[0] & 0x80)
	{
		*ir = 0;                                                    /*IR_OF位为1，则数据无效 */
	}
	else
	{
		*ir = ((uint16)buf[1] << 2) | (buf[0] & 0x03);              /* 读取IR传感器的数据 */
	}
	
	*als = ((uint16)buf[3] << 8) | buf[2];                        /* 读取ALS传感器的数据 */
	
	if(buf[4] & 0x40)
	{
		*ps = 0;
	}
	else
	{
		*ps = ((uint16)(buf[5] & 0x3F) << 4) |(buf[4] & 0x0F);      /* 读取PS传感器的数据 */
	}
}

uint8 ap3216c_write_one_byte(uint8 reg, uint8 data)
{
	My_I2C_Start();
	My_I2C_WriteByte(AP3216C_ADDR | 0X00);      /*发送器件地址+写命令 */
	
	if(My_I2C_Wait_Ack())
	{
		My_I2C_Stop();
		return 1;
	}
	
	My_I2C_WriteByte(reg);
	My_I2C_Wait_Ack();
	My_I2C_WriteByte(data);

  if(My_I2C_Wait_Ack())	
	{
		My_I2C_Stop();
		return 1;
	}
	  My_I2C_Stop();
		return 0;
	
}

uint8 ap3216c_read_one_byte(uint8 reg)
{
	uint8 res;
	
	My_I2C_Start();
	My_I2C_WriteByte(AP3216C_ADDR | 0X00);
	My_I2C_Wait_Ack();
	My_I2C_WriteByte(reg);
	My_I2C_Wait_Ack();
	My_I2C_Start();
	
	My_I2C_WriteByte(AP3216C_ADDR | 0X01);
	My_I2C_Wait_Ack();
	res = My_I2C_ReadByte(0);
	My_I2C_Stop();
	
	return res;
}
