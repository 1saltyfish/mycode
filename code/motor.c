#include "zf_common_headfile.h"
#include "motor.h" 
#define CHANNEL_NUMBER          (4)

#define PWM_CH1                 (TIM5_PWM_CH1_A0)
#define PWM_CH2                 (TIM5_PWM_CH2_A1)
#define PWM_CH3                 (TIM5_PWM_CH3_A2)
#define PWM_CH4                 (TIM5_PWM_CH4_A3)

int16 duty = 0;
uint8 channel_index = 0;
pwm_channel_enum channel_list[CHANNEL_NUMBER] = {PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4};

void Motor_PWM_Init()
{
                                                                 // ��ʼ��Ĭ�� Debug UART
//    timer_init(TIM_5, TIMER_SYSTEM_CLOCK);                                 
//    timer_start(TIM_5);
    pwm_init(PWM_CH1, 10000, 0);                                                // ��ʼ�� PWM ͨ�� Ƶ�� 10KHz ��ʼռ�ձ� 0%
    pwm_init(PWM_CH2, 10000, 0);                                                // ��ʼ�� PWM ͨ�� Ƶ�� 10KHz ��ʼռ�ձ� 0%
    pwm_init(PWM_CH3, 10000, 0);                                                // ��ʼ�� PWM ͨ�� Ƶ�� 10KHz ��ʼռ�ձ� 0%
    pwm_init(PWM_CH4, 10000, 0);                                                // ��ʼ�� PWM ͨ�� Ƶ�� 10KHz ��ʼռ�ձ� 0% 
}

void Motor_GPIO_Init(void)
{
	
  
    gpio_init(B8, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED1 ���  �������ģʽ
    gpio_init(B9, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED2 ���  �������ģʽ
	
    gpio_init(B0, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED1 ���  �������ģʽ
    gpio_init(B1, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED2 ���  �������ģʽ
	  gpio_init(G0, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED1 ���  �������ģʽ
    gpio_init(G1, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED2 ���  �������ģʽ
	
    gpio_init(G2, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED1 ���  �������ģʽ
    gpio_init(G3, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // ��ʼ�� LED2 ���  �������ģʽ
}

void forward(float speed1,float speed2,float speed3,float speed4)
{
	pwm_set_duty(PWM_CH1,speed1);//����
	RAIN1_ON;
	RAIN2_OFF;
	
	pwm_set_duty(PWM_CH2,speed2);//����
	RBIN1_ON;
	RBIN2_OFF;
	
  pwm_set_duty(PWM_CH3,speed3 );//����
	LAIN1_ON;
	LAIN2_OFF;
	
	pwm_set_duty(PWM_CH4,speed4 );//����
	LBIN1_ON;
	LBIN2_OFF;
	
}

void backward(float speed1,float speed2,float speed3,float speed4)
{
  pwm_set_duty(PWM_CH1,speed1);//����
	RAIN1_OFF;
	RAIN2_ON;
	
	pwm_set_duty(PWM_CH2,speed2 );//����
	RBIN1_OFF;
	RBIN2_ON;
	
	pwm_set_duty(PWM_CH3,speed3 );//����
	LAIN1_OFF;
	LAIN2_ON;
	
	pwm_set_duty(PWM_CH4,speed4 );//����
	LBIN1_OFF;
	LBIN2_ON;
}

void Right_turn(float speed1,float speed2,float speed3,float speed4)
{
	pwm_set_duty(PWM_CH1,speed1 );//����
	RAIN1_ON;
	RAIN2_OFF;
	
	pwm_set_duty(PWM_CH2,speed2 );//����
	RBIN1_ON;
	RBIN2_OFF;
	
	pwm_set_duty(PWM_CH3,speed3 );//����
	LAIN1_OFF;
	LAIN2_ON;
	
	pwm_set_duty(PWM_CH4,speed4 );//����
	LBIN1_OFF;
	LBIN2_ON;
}
void left_turn(float speed1,float speed2,float speed3,float speed4)
{
	pwm_set_duty(PWM_CH1,speed1 );//����
	RAIN1_OFF;
	RAIN2_ON;
	
	pwm_set_duty(PWM_CH2,speed2 );//����
	RBIN1_OFF;
	RBIN2_ON;
	
	pwm_set_duty(PWM_CH3,speed3 );//����
	LAIN1_ON;
	LAIN2_OFF;
	
	pwm_set_duty(PWM_CH4,speed4 );//����
	LBIN1_ON;
	LBIN2_OFF;
}
/**************************************************
�������ƣ�Left_Turn(u16 speed)
�������ܣ�С������
��ڲ�����speed  0-500
���ز�������
***************************************************/
void Right_move(float speed1,float speed2,float speed3,float speed4)
{
	pwm_set_duty(PWM_CH1,speed1 );//����
	RAIN1_ON;
	RAIN2_OFF;
	
	pwm_set_duty(PWM_CH2,speed2 );//���
	RBIN1_OFF;
	RBIN2_ON;
	
		pwm_set_duty(PWM_CH3,speed3 );//�Һ�
	LAIN1_ON;
	LAIN2_OFF;
	
	pwm_set_duty(PWM_CH4,speed4 );//��ǰ
	LBIN1_OFF;
	LBIN2_ON;
}


/**************************************************
�������ƣ�Right_Turn(u16 speed)
�������ܣ�С��ZUO��
��ڲ�����speed  0-500
���ز�������
***************************************************/
void Left_move(float speed1,float speed2,float speed3,float speed4)
{
	pwm_set_duty(PWM_CH1,speed1 );//����
	RAIN1_OFF;
	RAIN2_ON;
	
	pwm_set_duty(PWM_CH2,speed2 );
	RBIN1_ON;
	RBIN2_OFF;
	
	pwm_set_duty(PWM_CH3,speed3 );
	LAIN1_OFF;
	LAIN2_ON;
	
	pwm_set_duty(PWM_CH4,speed4 );
	LBIN1_ON;
	LBIN2_OFF;
}
void stop_move()
{
	pwm_set_duty(PWM_CH1,0 );//����
	RAIN1_OFF;
	RAIN2_OFF;
	
	pwm_set_duty(PWM_CH2,0 );//����
	RBIN1_OFF;
	RBIN2_OFF;
	
	pwm_set_duty(PWM_CH3,0 );//����
	LAIN1_OFF;
	LAIN2_OFF;
	
	pwm_set_duty(PWM_CH4,0);//����
	LBIN1_OFF;
	LBIN2_OFF;
}



