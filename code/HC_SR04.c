#include "zf_common_headfile.h"
#include "HC_SR04.h"

uint8 msHcCount = 0;//ms����


uint32_t my_GetUpdateInterruptStatus(TIM_Type *TIMx)
{
    uint32_t status = TIMx->SR;  // ��ȡ�����ж�״̬�Ĵ�����ֵ
    uint32_t updateStatus = status & TIM_STATUS_UPDATE_PERIOD; // ʹ��λ���������������Ҫ��λ
    
    return updateStatus;  // �����жϸ��±�־λ��״̬
}

void Hcsr04Init()
{  
		gpio_init(E10, GPO, GPIO_LOW,  GPO_PUSH_PULL); //trig�˿�
		gpio_init(E15, GPI, GPIO_LOW, 	GPI_PULL_DOWN);
  	system_delay_us(15);
	  pit_ms_init(TIM2_PIT,38);   
	  TIM_SetClockDiv((TIM_Type * )TIM2, TIM_ClockDiv_Alt0);	// ��ʼ�� PIT Ϊ�����ж� 1000us ����
    interrupt_set_priority(TIM2_IRQn, 2);                                    // ���� PIT �������жϵ��ж����ȼ�Ϊ 0
	  timer_stop(TIM_2);
	  TIM_ClearInterruptStatus((TIM_Type *)TIM2, TIM_GetInterruptStatus((TIM_Type *)TIM2));
	
		//ʱ�ӷ�Ƶ����
//		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;          
}

void initHcsr04()
{
	Hcsr04Init();
}


static void OpenTimer()        //�򿪶�ʱ��
{
        //	/*���������*/
	timer_clear(TIM_2);
	msHcCount = 0;
	timer_start(TIM_2);//ʹ�ܶ�ʱ��
}
 
static void CloseTimer()        //�رն�ʱ��
{
       //	/*�رռ�����ʹ��*/
	timer_stop(TIM_2);
}
 

//��ʱ��2�жϷ������
//void TIM2_IRQHandler(void)
//{
//        //	/*�ж��ж��ַ���Ĳ���*/
////	if(my_GetUpdateInterruptStatus((TIM_Type *)TIM2) == 0)
////	{
////		TIM_ClearInterruptStatus((TIM_Type *)TIM2, TIM_STATUS_UPDATE_PERIOD);
////		msHcCount++;
////	}
//}

//��ʱ��2�жϷ������
void TIM2_IRQHandler (void)
{
    msHcCount++;
    TIM_ClearInterruptStatus((TIM_Type *)TIM2, TIM_GetInterruptStatus((TIM_Type *)TIM2));
}


//��ȡ��ʱ��ʱ��
uint32 GetEchoTimer(void)
{
   uint32 time = 0;
	/*//�������źźܳ��ǣ�����ֵ������ظ�������overCount���ж��������������*/
	time = msHcCount * 1000;//overCountÿ++һ�Σ�����overCount���룬time΢��
	time += timer_get(TIM_2);;//��ȡ��TIM2���Ĵ����еļ���ֵ��һ�߼�������ź�ʱ��
	timer_clear(TIM_2);  //��TIM2�����Ĵ����ļ���ֵ����
	system_delay_ms(50);
	return time;
 
}
float Hcsr04GetLength(void )
{
	/*��5�����ݼ���һ��ƽ��ֵ*/
	float length = 0;
	float t = 0;
	float sum = 0;
	uint16  	i = 0;
	while(i != 5){
		gpio_set_level(E10, 1);//trig�����źţ������ߵ�ƽ
		system_delay_us(20);//����ʱ�䳬��10us
		gpio_set_level(E10, 0);
		/*Echo�����ź� �ȴ������ź�*/
		/*���뷽����ģ����Զ�����8��40KHz�����������ͬʱ�ز����ţ�echo���˵ĵ�ƽ����0��Ϊ1��
		����ʱӦ��������ʱ����ʱ���������������ر�ģ����յ�ʱ���ز��� �Ŷ˵ĵ�ƽ����1��Ϊ0��
		����ʱӦ��ֹͣ��ʱ������������ʱ�����µ����ʱ�伴Ϊ
			�������ɷ��䵽���ص���ʱ����*/
		while(gpio_get_level(E15) == 0);//echo�ȴ�����
		/*������ʱ��*/
		OpenTimer();
		i = i+1; //ÿ�յ�һ�λ����ź�+1���յ�5�ξͼ����ֵ
		while(gpio_get_level(E15) == 1);
		/*�رն�ʱ��*/
		CloseTimer();
		/*��ȡEcho�ߵ�ƽʱ��ʱ��*/
		t = GetEchoTimer();
		length = (float)t/58;//��λʱcm
		sum += length;		
	}
	length = sum/5;//���ƽ��ֵ
	
	return length;
}