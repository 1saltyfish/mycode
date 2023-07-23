#include "zf_common_headfile.h"
#include "motor.h" 
#include "math.h"
int t4=0;
int pwmout = 0;                                 // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体
float setspeed=5000;                                 // 对应外部中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体
//float kp=1.09,ki=0.00001,kd=0;
float kp=1.18,ki=0.0,kd=0.11;
float error_i=0;
extern float motorspeed,motorspeed2,motorspeed3,motorspeed4;
float  error = 0;
static int errorold = 0,errorold2 = 0;
int x;
float myabs(float a)
{ 		   
	  float temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
float pid1(float motor,float set)
{
    motorspeed=myabs(motorspeed);
     error=setspeed-motorspeed;
    pwmout +=kp*(error-errorold)+ki*(error)+kd*(error-2*errorold+errorold2);
	  if(pwmout>9800)pwmout=9800;
	  if(pwmout<-9800)pwmout=-9800;
	   errorold2=errorold;
	   errorold=error;
    return pwmout;
}
float pid2(float motor,float set)
{
    motorspeed2=myabs(motorspeed2);
     error=setspeed-motorspeed2;
    pwmout +=kp*(error-errorold)+ki*(error)+kd*(error-2*errorold+errorold2);
	  if(pwmout>9800)pwmout=9800;
	  if(pwmout<-9800)pwmout=-9800;
	   errorold2=errorold;
	   errorold=error;
    return pwmout;
}
float pid3(float motor,float set)
{
    motorspeed3=myabs(motorspeed3);
     error=setspeed-motorspeed3;
    pwmout +=kp*(error-errorold)+ki*(error)+kd*(error-2*errorold+errorold2);
	  if(pwmout>9800)pwmout=9800;
	  if(pwmout<-9800)pwmout=-9800;
	   errorold2=errorold;
	   errorold=error;
    return pwmout;
}
float pid4(float motor,float set)
{
    motorspeed4=myabs(motorspeed4);
     error=setspeed-motorspeed4;
    pwmout +=kp*(error-errorold)+ki*(error)+kd*(error-2*errorold+errorold2);
	  if(pwmout>9800)pwmout=9800;
	  if(pwmout<-9800)pwmout=-9800;
	   errorold2=errorold;
	   errorold=error;
    return pwmout;
}