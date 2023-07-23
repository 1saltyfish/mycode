#include "AP3216.h"
#include "MyIIC.h"
#include "zf_common_headfile.h"

uint8 ap3216c_init(void)
{
	uint8 temp = 0;
	
	My_IIC_GPIO_Init();
	ap3216c_write_one_byte(0x00, 0x04);  /*��λAP3216C */
	system_delay_ms (50);                /* AP3216C��λ����10ms */
	ap3216c_write_one_byte(0x00, 0x03);  /* ����ALS��PS+IR */
	
	temp = ap3216c_read_one_byte(0x00);  /* ��ȡ�ո�д��ȥ��0x03 */
	if(temp == 0x03)
	{
		return 0;    /* AP3216C���� */
	}
	else
	{
		return 1;    /* AP33216Cʧ�� */
	}
}

void ap3216c_read_data(uint16 *ir, uint16 *ps, uint16_t *als)
{
	uint8 buf[6];
	uint8 i;
	
	for(i = 0; i < 6; i++)
	{
		buf[i] = ap3216c_read_one_byte(0X0A + i);                   /* ѭ����ȡ���д��������� */
	}
	
	if(buf[0] & 0x80)
	{
		*ir = 0;                                                    /*IR_OFλΪ1����������Ч */
	}
	else
	{
		*ir = ((uint16)buf[1] << 2) | (buf[0] & 0x03);              /* ��ȡIR������������ */
	}
	
	*als = ((uint16)buf[3] << 8) | buf[2];                        /* ��ȡALS������������ */
	
	if(buf[4] & 0x40)
	{
		*ps = 0;
	}
	else
	{
		*ps = ((uint16)(buf[5] & 0x3F) << 4) |(buf[4] & 0x0F);      /* ��ȡPS������������ */
	}
}

uint8 ap3216c_write_one_byte(uint8 reg, uint8 data)
{
	My_I2C_Start();
	My_I2C_WriteByte(AP3216C_ADDR | 0X00);      /*����������ַ+д���� */
	
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
