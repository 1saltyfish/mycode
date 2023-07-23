#ifndef __DHT11_2_H
#define __DHT11_2_H
#include "zf_common_headfile.h"



#define dht11_high gpio_set_level(B13, 1)
#define dht11_low gpio_set_level(B13, 0)
#define Read_Data gpio_get_level(B13)

void DHT11_GPIO_Init_OUT(void);
void DHT11_GPIO_Init_IN(void);
void DHT11_Start(void);
unsigned char DHT11_REC_Byte(void);
void DHT11_REC_Data(void);



#endif
