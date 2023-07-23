#ifndef __AP3216_H
#define __AP3216_H

#include "MyIIC.h"
#include "zf_common_headfile.h"

#define AP3216C_ADDR    0x3C     /* AP3216C器件IIC地址（左移了一位） */

uint8 ap3216c_init(void);
uint8 ap3216c_write_one_byte(uint8 reg, uint8 data);
uint8 ap3216c_read_one_byte(uint8 reg);
void ap3216c_read_data(uint16* ir,uint16* ps, uint16* als);

#endif