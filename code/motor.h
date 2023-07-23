#ifndef __MOTOR_H
#define __MOTOR_H	 
#include "zf_common_headfile.h"
#include "motor.h" 


#define RAIN1_ON    gpio_set_level(B8, GPIO_HIGH) //输出高电平
#define RAIN1_OFF   gpio_set_level(B8, GPIO_LOW) //输出高电平
#define RAIN2_ON 	gpio_set_level(B9, GPIO_HIGH)    //输出高电平
#define RAIN2_OFF  gpio_set_level(B9, GPIO_LOW)  //输出低电平


#define RBIN1_ON 	gpio_set_level(B0, GPIO_HIGH)    //输出高电平
#define RBIN1_OFF  gpio_set_level(B0, GPIO_LOW)  //输出低电平
#define RBIN2_ON 	gpio_set_level(B1, GPIO_HIGH)    //输出高电平
#define RBIN2_OFF  gpio_set_level(B1, GPIO_LOW)  //输出低电平

#define LAIN1_ON    gpio_set_level(G0, GPIO_HIGH) //输出高电平
#define LAIN1_OFF   gpio_set_level(G0, GPIO_LOW) //输出高电平
#define LAIN2_ON 	gpio_set_level(G1, GPIO_HIGH)    //输出高电平
#define LAIN2_OFF  gpio_set_level(G1, GPIO_LOW)  //输出低电平


#define LBIN1_ON 	gpio_set_level(G2, GPIO_HIGH)    //输出高电平
#define LBIN1_OFF  gpio_set_level(G2, GPIO_LOW)  //输出低电平
#define LBIN2_ON 	gpio_set_level(G3, GPIO_HIGH)    //输出高电平
#define LBIN2_OFF  gpio_set_level(G3, GPIO_LOW)  //输出低电平
#define speed  speed 

void Motor_PWM_Init();
void Motor_GPIO_Init();
void forward(float speed1,float speed2,float speed3,float speed4);
void backward(float speed1,float speed2,float speed3,float speed4);
void Right_turn(float speed1,float speed2,float speed3,float speed4);
void left_turn(float speed1,float speed2,float speed3,float speed4);
void Right_move(float speed1,float speed2,float speed3,float speed4);
void Left_move(float speed1,float speed2,float speed3,float speed4);
void stop_move();
#endif

