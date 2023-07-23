#include "SGP30.h"
#include "zf_common_headfile.h"


void SGP30_GPIO_Init(void)
{
	gpio_init(B15, GPO, GPIO_HIGH, GPO_PUSH_PULL); /* SCL */
	gpio_init(B14, GPO, GPIO_HIGH, GPO_PUSH_PULL); /* SDA */
}

void SDA_OUT(void)
{
  gpio_init(B14, GPO, GPIO_HIGH, GPO_PUSH_PULL); /* SDA */ 
}

void SDA_IN(void)
{
	gpio_init(B14, GPI, GPIO_HIGH, GPI_FLOATING_IN); /* SDA */
}

void SGP30_IIC_Start(void)
{
  SDA_OUT();
  SGP30_SDA_SET; 
  SGP30_SCL_SET;
  system_delay_us(20);

  SGP30_SDA_RESET;
  system_delay_us(20);
  SGP30_SCL_RESET;
}

void SGP30_IIC_Stop(void)
{
  SDA_OUT();
  SGP30_SCL_RESET;
  SGP30_SDA_RESET;	
  system_delay_us(20);
  SGP30_SCL_SET;
  SGP30_SDA_SET;
  system_delay_us(20);
}

unsigned char SGP30_IIC_Wait_Ack(void)
{
  unsigned char ucErrTime = 0;
  SDA_IN();
	
  SGP30_SDA_SET;
  system_delay_us(10);
  SGP30_SCL_SET;
  system_delay_us(10);
	
  while(SGP30_SDA_READ())
  {
    ucErrTime++;
    if(ucErrTime > 250)
    {
      SGP30_IIC_Stop();
      return 1;
    }
  }
  SGP30_SCL_RESET; 
  return 0;
}

void SGP30_IIC_Ack(void)
{
  SGP30_SCL_RESET;
  SDA_OUT();
  SGP30_SDA_RESET;
  system_delay_us(20);
  SGP30_SCL_SET;
  system_delay_us(20);
  SGP30_SCL_RESET;
}

void SGP30_IIC_NAck(void)
{
  SGP30_SCL_RESET;
  SDA_OUT();
  SGP30_SDA_SET;
  system_delay_us(20);
  SGP30_SCL_SET;
  system_delay_us(20);
  SGP30_SCL_RESET;
}

void SGP30_IIC_Send_Byte(uint8 txd)
{
  uint8 t;
  SDA_OUT();
  SGP30_SCL_RESET; 
  for(t = 0; t < 8; t++)
  {
    if((txd & 0x80) >> 7)
      SGP30_SDA_SET;
    else
      SGP30_SDA_RESET;
    txd <<= 1;
    system_delay_us(20);
    SGP30_SCL_SET;
    system_delay_us(20);
    SGP30_SCL_RESET;
    system_delay_us(20);
  }
  system_delay_us(20);

}

unsigned int SGP30_IIC_Read_Byte(uint8 ack)
{
  uint8 i;
  uint16 receive = 0;
  SDA_IN();
  for(i = 0; i < 8; i++ )
  {
    SGP30_SCL_RESET;
    system_delay_us(20);
    SGP30_SCL_SET;
    receive <<= 1;
    if(SGP30_SDA_READ())
      receive++;
    system_delay_us(20);
  }
  if (!ack)
    SGP30_IIC_NAck();
  else
    SGP30_IIC_Ack();
  return receive;
}

void SGP30_Init(void)
{
  SGP30_GPIO_Init();
  SGP30_Write(0x20, 0x03);
//	SGP30_ad_write(0x20,0x61);
//	SGP30_ad_write(0x01,0x00);
}


void SGP30_Write(uint8 a, uint8 b)
{
  SGP30_IIC_Start();
  SGP30_IIC_Send_Byte(SGP30_write); 
  SGP30_IIC_Wait_Ack();
  SGP30_IIC_Send_Byte(a);	
  SGP30_IIC_Wait_Ack();
  SGP30_IIC_Send_Byte(b);
  SGP30_IIC_Wait_Ack();
  SGP30_IIC_Stop();
  system_delay_ms(100);
}

uint32 SGP30_Read(void)
{
  uint32 dat;
  uint8 crc;
  SGP30_IIC_Start();
  SGP30_IIC_Send_Byte(SGP30_read); 
  SGP30_IIC_Wait_Ack();
  dat = SGP30_IIC_Read_Byte(1);
  dat <<= 8;
  dat += SGP30_IIC_Read_Byte(1);
  crc = SGP30_IIC_Read_Byte(1); 
  crc = crc;
  dat <<= 8;
  dat += SGP30_IIC_Read_Byte(1);
  dat <<= 8;
  dat += SGP30_IIC_Read_Byte(0);
  SGP30_IIC_Stop();
  return(dat);
}
