#ifndef __PID_H
#define __PID_H	 
#include "zf_common_headfile.h"
#include "motor.h" 

float pid1(float motor,float set);
float pid2(float motor,float set);
float pid3(float motor,float set);
float pid4(float motor,float set);
 
#endif
