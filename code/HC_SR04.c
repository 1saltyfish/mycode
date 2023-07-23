#include "zf_common_headfile.h"
#include "HC_SR04.h"

uint8 msHcCount = 0;//ms计数


uint32_t my_GetUpdateInterruptStatus(TIM_Type *TIMx)
{
    uint32_t status = TIMx->SR;  // 获取整个中断状态寄存器的值
    uint32_t updateStatus = status & TIM_STATUS_UPDATE_PERIOD; // 使用位掩码操作仅保留需要的位
    
    return updateStatus;  // 返回中断更新标志位的状态
}

void Hcsr04Init()
{  
		gpio_init(E10, GPO, GPIO_LOW,  GPO_PUSH_PULL); //trig端口
		gpio_init(E15, GPI, GPIO_LOW, 	GPI_PULL_DOWN);
  	system_delay_us(15);
	  pit_ms_init(TIM2_PIT,38);   
	  TIM_SetClockDiv((TIM_Type * )TIM2, TIM_ClockDiv_Alt0);	// 初始化 PIT 为周期中断 1000us 周期
    interrupt_set_priority(TIM2_IRQn, 2);                                    // 设置 PIT 对周期中断的中断优先级为 0
	  timer_stop(TIM_2);
	  TIM_ClearInterruptStatus((TIM_Type *)TIM2, TIM_GetInterruptStatus((TIM_Type *)TIM2));
	
		//时钟分频因子
//		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;          
}

void initHcsr04()
{
	Hcsr04Init();
}


static void OpenTimer()        //打开定时器
{
        //	/*清除计数器*/
	timer_clear(TIM_2);
	msHcCount = 0;
	timer_start(TIM_2);//使能定时器
}
 
static void CloseTimer()        //关闭定时器
{
       //	/*关闭计数器使能*/
	timer_stop(TIM_2);
}
 

//定时器2中断服务程序
//void TIM2_IRQHandler(void)
//{
//        //	/*判断中断手否真的产生*/
////	if(my_GetUpdateInterruptStatus((TIM_Type *)TIM2) == 0)
////	{
////		TIM_ClearInterruptStatus((TIM_Type *)TIM2, TIM_STATUS_UPDATE_PERIOD);
////		msHcCount++;
////	}
//}

//定时器2中断服务程序
void TIM2_IRQHandler (void)
{
    msHcCount++;
    TIM_ClearInterruptStatus((TIM_Type *)TIM2, TIM_GetInterruptStatus((TIM_Type *)TIM2));
}


//获取定时器时间
uint32 GetEchoTimer(void)
{
   uint32 time = 0;
	/*//当回响信号很长是，计数值溢出后重复计数，overCount用中断来保存溢出次数*/
	time = msHcCount * 1000;//overCount每++一次，代表overCount毫秒，time微妙
	time += timer_get(TIM_2);;//获取计TIM2数寄存器中的计数值，一边计算回响信号时间
	timer_clear(TIM_2);  //将TIM2计数寄存器的计数值清零
	system_delay_ms(50);
	return time;
 
}
float Hcsr04GetLength(void )
{
	/*测5次数据计算一次平均值*/
	float length = 0;
	float t = 0;
	float sum = 0;
	uint16  	i = 0;
	while(i != 5){
		gpio_set_level(E10, 1);//trig拉高信号，发出高电平
		system_delay_us(20);//持续时间超过10us
		gpio_set_level(E10, 0);
		/*Echo发出信号 等待回响信号*/
		/*输入方波后，模块会自动发射8个40KHz的声波，与此同时回波引脚（echo）端的电平会由0变为1；
		（此时应该启动定时器计时）；当超声波返回被模块接收到时，回波引 脚端的电平会由1变为0；
		（此时应该停止定时器计数），定时器记下的这个时间即为
			超声波由发射到返回的总时长；*/
		while(gpio_get_level(E15) == 0);//echo等待回响
		/*开启定时器*/
		OpenTimer();
		i = i+1; //每收到一次回响信号+1，收到5次就计算均值
		while(gpio_get_level(E15) == 1);
		/*关闭定时器*/
		CloseTimer();
		/*获取Echo高电平时间时间*/
		t = GetEchoTimer();
		length = (float)t/58;//单位时cm
		sum += length;		
	}
	length = sum/5;//五次平均值
	
	return length;
}