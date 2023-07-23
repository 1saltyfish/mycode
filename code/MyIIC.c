#include "zf_common_headfile.h"
#include "MyIIC.h"

void My_IIC_GPIO_Init(void)
{
	gpio_init(B12, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);	
	gpio_init(E4, GPO, GPIO_HIGH, GPO_OPEN_DTAIN); 
	
	I2C_SDA_SET();
	I2C_SCL_SET();
}

 void My_I2C_Start(void)
{
	I2C_SDA_SET();
	I2C_SCL_SET();
	system_delay_us(2);
	
	I2C_SDA_CLR();
	system_delay_us(2);
	
	I2C_SCL_CLR();
	system_delay_us(2);
}

void My_I2C_Stop(void)
{
	I2C_SDA_CLR();
	system_delay_us(2);
	
	I2C_SCL_SET();
	system_delay_us(2);
	
	I2C_SDA_SET();
	system_delay_us(2);
}

unsigned char My_I2C_Wait_Ack(void)
{
	I2C_SDA_SET();
	system_delay_us(2);
	I2C_SCL_SET();
	system_delay_us(2);
	if(I2C_SDA_Read())
	{
		My_I2C_Stop();
		return 1;
	}
	
	I2C_SCL_CLR();
	system_delay_us(2);
	return 0;
}

void My_I2C_Ack(void)
{
	I2C_SCL_CLR();
	system_delay_us(2);
	
	I2C_SDA_CLR();
	system_delay_us(2);
	
	I2C_SCL_SET();
	system_delay_us(2);	
}

void My_I2C_Nack()
{
  I2C_SCL_CLR();
	system_delay_us(2);
	
	I2C_SDA_SET();
	system_delay_us(2);
	
	I2C_SCL_SET();
	system_delay_us(2);
}

void My_I2C_WriteByte(unsigned char I2C_Byte)
{
	for(uint8 t = 0; t < 8; t++)
	{
		gpio_set_level(E4,((I2C_Byte & 0x80) >> 7));
		system_delay_us(2);
		I2C_SCL_SET();
		system_delay_us(2);
		I2C_SCL_CLR();
		I2C_Byte <<= 1;
	}
	I2C_SDA_SET();	
}

unsigned char My_I2C_ReadByte(uint8 ack)
{
	uint8 receive = 0;
	for(uint8 t = 0; t < 8; t++)
	{
		receive <<= 1;
		I2C_SCL_SET();
		system_delay_us(2);
		if(I2C_SDA_Read()) receive++;
		I2C_SCL_CLR();
		system_delay_us(2);
	}
	if( !ack ) My_I2C_Nack();
	else My_I2C_Ack();
	return receive;
}
