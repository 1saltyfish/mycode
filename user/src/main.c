#include "zf_common_headfile.h"
#include "DHT11_2.h"
#include "SGP30.h"
#include "AP3216.h"
#include "wit_c_sdk.h"
#include "motor.h"
#include "HC_SR04.h"

#define WIFI_SSID_TEST          "1111"
#define WIFI_PASSWORD_TEST      "12345678"
#define LED1                    (H2 )

#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_1
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART1_TX_A9
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART1_RX_A10
#define UART_PRIORITY           (UART1_IRQn)                                    // 对应串口中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

/*************************************************舵机模块*******************************************************************/
#define PWM_CH1                 (TIM1_PWM_CH1_A8)
/****************************************************************************************************************************/

/*************************************************电机驱动模块*******************************************************************/
#define PIT                     (TIM6_PIT )                                     // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY            (TIM6_IRQn)  

int t4=0;
float pwmout = 0, pwmout2 = 0, pwmout3 = 0, pwmout4 = 0;                                 // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体
float setspeed1 = 75, setspeed2 = 75, setspeed3 = 75, setspeed4 = 75;
float motorspeed = 0, motorspeed2 = 0, motorspeed3 = 0, motorspeed4 = 0;                                  // 对应外部中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

float kp = 0.12, ki = 0.7, kd = 0;
float kp2 = 0.12, ki2 = 0.7, kd2 = 0;
float kp3 = 0.12, ki3 = 0.7, kd3 = 0;
float kp4 = 0.12, ki4 = 0.7, kd4 = 0;

float  error = 0, errorx = 0, errory=0, errorz=0;
static int errorold = 0, errorold2 = 0;
static int erroroldx = 0, errorold2x = 0;
static int erroroldy = 0, errorold2y = 0;
static int erroroldz = 0, errorold2z = 0;
int16 count = 0, count1 = 0;
/********************************************************************************************************************************/

/**************************************************超声波测距模块****************************************************************/
void avoid_barrier_forward();
void avoid_barrier_backward();
void avoid_barrier_leftmove();
void avoid_barrier_rightmove();
/********************************************************************************************************************************/

/*********************************************************姿态传感器模块*********************************************************************/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
void my_usart1_init(void);
void my_usart5_init(void);
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
void control_way(void);

float fAcc[3], fGyro[3], fAngle[3];
uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8 get_data = 0;                                                             // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;
/********************************************************************************************************************************/

/*************************************************电机驱动模块*******************************************************************/
float myabs(float a)
{ 		   
	  float temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

float pid1(float set1)
{
    motorspeed=myabs(motorspeed);
     error=set1-motorspeed;
    pwmout +=kp*(error-errorold)+ki*(error)+kd*(error-2*errorold+errorold2);
	  if(pwmout>193)pwmout=193;
	  if(pwmout<-193)pwmout=-193;
	   errorold2=errorold;
	   errorold=error;
    return pwmout;
}

float pid2(float set2)
{
    motorspeed2=myabs(motorspeed2);
     errorx=set2-motorspeed2;
    pwmout2 +=kp2*(errorx-erroroldx)+ki2*(errorx)+kd2*(errorx-2*erroroldx+errorold2x);
	  if(pwmout2>193)pwmout2=193;
	  if(pwmout2<-193)pwmout2=-193;
	   errorold2x=erroroldx;
	   erroroldx=errorx;
    return pwmout2;
}

float pid3(float set3)
{
    motorspeed3=myabs(motorspeed3);
     errory=set3-motorspeed3;
    pwmout3 +=kp3*(errory-erroroldy)+ki3*(errory)+kd3*(errory-2*erroroldy+errorold2y);
	  if(pwmout3>193)pwmout3=193;
	  if(pwmout3<-193)pwmout3=-193;
	   errorold2y=erroroldy;
	   erroroldy=errory;
    return pwmout3;
}

float pid4(float set4)
{
    motorspeed4=myabs(motorspeed4);
     errorz=set4-motorspeed4;
    pwmout4 +=kp4*(errorz-erroroldz)+ki4*(errorz)+kd4*(errorz-2*erroroldz+errorold2z);
	  if(pwmout4>193)pwmout4=193;
	  if(pwmout4<-193)pwmout4=-193;
	   errorold2z=erroroldz;
	   erroroldz=errorz;
    return pwmout4;
}
/********************************************************************************************************************************/

extern uint16 rec_data[4];
float length = 0;

uint8 wifi_uart_test_buffer[] = "this is wifi uart test buffer";
uint8 wifi_uart_get_buffer[256];
uint8 wifi_uart_get_data_buffer[512];
uint16 data_length;
uint16 dataa;
uint16 data;
uint8 longString[256];
uint8 key;
uint16 a1= 0, b1 = 0, c1 = 0;
uint8 flag1 = 1, flag2 = 1, flag3 = 1, flag4 = 1, flag5 = 1, flag6 = 1;
uint8 mode = 1;

void esp8266_get(uint8 f)
{
	sprintf((char*)longString,"GET /devices/1101718716/datapoints?datastream_id=key HTTP/1.1\r\n"\
	"api-key:zWoyl5pjirtou34sUYsZpvSnu98=\r\n"\
	"Host:api.heclouds.com\n\r\n");
	wifi_uart_send_buffer(longString,sizeof(longString));
	system_delay_ms(1000);	
  data_length = wifi_uart_read_buffer(wifi_uart_get_data_buffer, sizeof(wifi_uart_get_data_buffer));
	
	if(data_length) 												        // 如果接收到数据 则进行数据类型判断
	{
			if(strstr((char *)wifi_uart_get_data_buffer, "\"value\":\"1\""))  		        // 判断数据格式是否是通过网络发送过来的数据
			{
				flag1 = 0; flag2 =1; flag3 =1; flag4 = 1; flag5 = 1; flag6 = 1; //前
			}
			if(strstr((char *)wifi_uart_get_data_buffer, "\"value\":\"2\""))  		        // 判断数据格式是否是通过网络发送过来的数据
			{
				flag1 = 1; flag2 =0; flag3 =1; flag4 = 1; flag5 = 1; flag6 = 1;  //后
			}
			if(strstr((char *)wifi_uart_get_data_buffer, "\"value\":\"4\""))  		        // 判断数据格式是否是通过网络发送过来的数据
			{
				flag1 = 1; flag2 =1; flag3 =0; flag4 = 1; flag5 = 1; flag6 = 1; //左平移
			}
			if(strstr((char *)wifi_uart_get_data_buffer, "\"value\":\"3\""))  		        // 判断数据格式是否是通过网络发送过来的数据
			{
				flag1 = 1; flag2 =1; flag3 =1; flag4 = 0; flag5 = 1; flag6 = 1;    //右平移
			}
			if(strstr((char *)wifi_uart_get_data_buffer, "\"value\":\"5\""))  		        // 判断数据格式是否是通过网络发送过来的数据
			{
        flag1 = 1; flag2 =1; flag3 =1; flag4 = 1; flag5 = 1; flag6 = 1;   //停止
			}
			if(strstr((char *)wifi_uart_get_data_buffer, "\"value\":\"6\""))  		        // 判断数据格式是否是通过网络发送过来的数据
			{
        mode = 1;
			}
	}
}

void GPS_post(void)
{
	 sprintf((char*)longString,"POST /devices/1101718716/datapoints HTTP/1.1\r\n"\
	"api-key:zWoyl5pjirtou34sUYsZpvSnu98=\r\n"\
	"Host:api.heclouds.com\r\n"\
	"Content-Length:100\r\n"\
	"\r\n"\
	"{\"datastreams\":[{\"id\":\"location\",\"datapoints\":[{\"value\":{\"lon\":%lf,\"lat\":%lf}}]}]}\r\n",gps_tau1201.latitude,gps_tau1201.longitude
					);
	 wifi_uart_send_buffer(longString,sizeof(longString));
}

void esp8266_post(uint8 t,uint8 h,uint16 c,uint16 l,uint16 tvoc)
{
	 sprintf((char*)longString,"POST /devices/1101718716/datapoints?type=3 HTTP/1.1\r\n"\
	"api-key:zWoyl5pjirtou34sUYsZpvSnu98=\r\n"\
	"Host:api.heclouds.com\r\n"\
	"Content-Length:100\r\n"\
	"\r\n"\
	 "{\"temp\":%d,\"humi\":%d,\"CO2\":%d,\"light\":%d,\"TVOC\":%d,\"latitude\":%lf,\"longitude\":%lf}\r\n",t,h,c,l,tvoc,
	 gps_tau1201.latitude,gps_tau1201.longitude);  							

   wifi_uart_send_buffer(longString,sizeof(longString));
}


void wifi_init(void)
{
    for(uint8 i = 0; i < 4; i++) length=Hcsr04GetLength();
		while(wifi_uart_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST, WIFI_UART_STATION));

}

uint32 CO2Data, TVOCData;  //定义CO2浓度变量与TVOC浓度变量
uint32 sgp30_dat;          //定义SGP30读取到的数据
uint16 t = 0, h = 0;
uint16 c = 0, tvoc = 0,l = 0;

int main (void)
{
	/***********************姿态传感器模块************************/
	  int i;
	/*************************************************/	
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
    SGP30_Init();
	  ap3216c_init(); 
	  initHcsr04();
	  pwm_init(PWM_CH1, 50, 0);
	  gps_init();
	  if(mode == 0) wifi_init(); 
	
	  encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_B4, TIM3_ENCODER_CH2_B5);
	  encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_D12,TIM4_ENCODER_CH2_D13);
	  exti_init(D4,EXTI_TRIGGER_BOTH);
	  gpio_init(D4, GPI, GPIO_HIGH, GPI_PULL_UP);
    interrupt_set_priority(EXTI4_IRQn, 3);
	  gpio_init(D7, GPI, GPIO_HIGH, GPI_PULL_UP);
	
		exti_init(D5,EXTI_TRIGGER_BOTH);
	  gpio_init(D5, GPI, GPIO_HIGH, GPI_PULL_UP);
    interrupt_set_priority(EXTI9_5_IRQn, 3);
	  gpio_init(D14, GPI, GPIO_HIGH, GPI_PULL_UP);

    gpio_init(A4, GPO, GPIO_HIGH, GPO_PUSH_PULL);
		gpio_init(A5, GPO, GPIO_HIGH, GPO_PUSH_PULL);
		gpio_init(E9, GPO, GPIO_LOW, GPO_PUSH_PULL);
		
		pit_ms_init(PIT,100);                   // 初始化 PIT 为周期中断 20ms 周期 	
    interrupt_set_priority(PIT_PRIORITY, 1); 
    Motor_PWM_Init();
    Motor_GPIO_Init();
	
	 /**********************姿态传感器模块*************************/
    my_usart1_init();                                                          // 初始化默认 debug uart
	  my_usart5_init();
		WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	  WitSerialWriteRegister(SensorUartSend);
	  WitRegisterCallBack(SensorDataUpdata);
	  WitDelayMsRegister(Delayms);
	  printf("\r\n wit-motion normal example \r\n");
	  AutoScanSensor();
	/*************************************************/
	
	  system_delay_ms(100);
	  SGP30_Write(0x20,0x08);
	  sgp30_dat = SGP30_Read();                  //读取SGP30的值
	  CO2Data = (sgp30_dat & 0xffff0000) >> 16;  //获取CO2的值
	  TVOCData = sgp30_dat & 0x0000ffff;         //获取TVOC的值
	  while(CO2Data == 400 && TVOCData == 0)
	 {
		SGP30_Write(0x20,0x08);
		sgp30_dat = SGP30_Read();                 //读取SGP30的值
		CO2Data = (sgp30_dat & 0xffff0000) >> 16; //获取CO2的值
		TVOCData = sgp30_dat & 0x0000ffff;		  //获取TVOC的值
		printf("waiting for testing...");
		system_delay_ms(500);
	 }
    while(1)
		{		
			if(mode == 0)
		 {
			while(wifi_uart_connect_tcp_servers(WIFI_UART_TARGET_IP, WIFI_UART_TARGET_PORT, WIFI_UART_COMMAND) == 0);
	    esp8266_get(key);
      while(wifi_uart_connect_tcp_servers(WIFI_UART_TARGET_IP, WIFI_UART_TARGET_PORT, WIFI_UART_COMMAND) == 0);			
    	esp8266_post(t,h,c,l,tvoc);
		wifi_init(); 		
		GPS_post();
			 if(gps_tau1201_flag)
			{
					gps_tau1201_flag = 0;

					if(!gps_data_parse())          //开始解析数据
					{
						system_delay_ms(10);
					}
			}
			
			if(flag1 == 0) avoid_barrier_forward();
			if(flag2 == 0) backward(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));
			if(flag3 == 0) Left_move(pid1(setspeed1)+10,pid2(setspeed2),pid3(setspeed3),pid4(setspeed4)+10);
			if(flag4 == 0) Right_move(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));
			if(flag1 == 1 && flag2 == 1 &&flag3 == 1 &&flag4 == 1 &&flag5 == 1 &&flag6 == 1) 
			stop_move();
		}
/************************************************姿态传感器模块**************************************************/				
				CmdProcess();
		if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE)
			{
//				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]); //加速度
				s_cDataUpdate &= ~ACC_UPDATE;
//				system_delay_ms(500);
			}
			if(s_cDataUpdate & GYRO_UPDATE)
			{
//				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]); //陀螺仪(旋转角速度)
				s_cDataUpdate &= ~GYRO_UPDATE;
//				system_delay_ms(500);
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
//				printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]); //角度
//				printf("angle:%.3f\r\n",fAngle[2]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
//				system_delay_ms(500);
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
//				printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]); //磁力计
				s_cDataUpdate &= ~MAG_UPDATE;
//				system_delay_ms(500);
			}
		}
/****************************************************************************************************/			
			ap3216c_read_data(&a1, &b1, &c1);
			DHT11_REC_Data();
			SGP30_Write(0x20,0x08);
		  sgp30_dat = SGP30_Read();                  //读取SGP30的值
		  CO2Data = (sgp30_dat & 0xffff0000) >> 16;  //获取CO2的值
		  TVOCData = sgp30_dat & 0x0000ffff;         //获取TVOC的值			
		  length=Hcsr04GetLength();	//获取距离	
			c= CO2Data; tvoc = TVOCData; 		
      h = rec_data[0]; t = rec_data[2];	
      l = c1;
		
			if(rec_data[2] > 26)  gpio_set_level(E9, 1);
		  else gpio_set_level(A5, 1);
		
     if(mode == 1) avoid_barrier_forward(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));		
		 printf("humidity:%d temperature:%d CO2:%d TVOC:%d light:%d dis:%fcm\r\n", h, t, c, tvoc, c1,length);
		printf("latitude: %lf longitude: %lf\r\n",gps_tau1201.latitude,gps_tau1201.longitude);	
	}
}

//---------------------------------------电机驱动模块-------------------------------------------//
void pit_handler (void)
{ 
	   if(CO2Data > 700 | TVOCData > 30)  gpio_toggle_level(A4);
		 else gpio_set_level(A4, 1);
		
		 if(c1 > 475)  gpio_toggle_level(A5);
		 else gpio_set_level(A5, 1);
	
	   motorspeed4 = count1;
     if(motorspeed4 < 0) motorspeed4 = -motorspeed4;
	   count1 = 0; 

	   motorspeed3 = count;
	   if(motorspeed3 < 0) motorspeed3 = -motorspeed3;
	   count = 0;

	   motorspeed2= encoder_get_count (TIM3_ENCODER);
     if(motorspeed2 < 0) motorspeed2 = -motorspeed2;	
	   encoder_clear_count(TIM3_ENCODER);
     TIM_ClearInterruptStatus((TIM_Type *)TIM3, TIM_GetInterruptStatus((TIM_Type *)TIM3));

	   motorspeed= encoder_get_count (TIM4_ENCODER);
     if(motorspeed < 0) motorspeed = -motorspeed;		 
	   encoder_clear_count(TIM4_ENCODER);
     TIM_ClearInterruptStatus((TIM_Type *)TIM4, TIM_GetInterruptStatus((TIM_Type *)TIM4));
}
//-------------------------------------------------------------------------------------------------//

//---------------------------------------USART1初始模块-------------------------------------------//
void my_usart1_init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区

    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
	  interrupt_set_priority(UART_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

    uart_write_string(UART_INDEX, "UART Text.");                                // 输出测试信息
    uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
    uart_write_byte(UART_INDEX, '\n');                                          // 输出换行			
}


void uart1_rx_interrupt_handler (void)
{
	unsigned char ucTemp;

	ucTemp = uart_read_byte(UART_1);
	WitSerialDataIn(ucTemp);
}

//-------------------------------------------------------------------------------------------------//

//---------------------------------------USART2初始模块-------------------------------------------//
void Uart5Send(unsigned char *p_data, unsigned int uiSize)
{
	unsigned int i;
	
	uint32_t status = UART_GetStatus(UART5);
  uint32_t status1 = UART_GetStatus(UART5);

  bool cts_status = (status & UART_CSR_TXEPT_MASK) >> UART_CSR_TXEPT_SHIFT;
  bool cts_status1 = (status1 & UART_CSR_TXC_MASK) >> UART_CSR_TXC_SHIFT;
	
	for(i = 0; i < uiSize; i++)
	{
		while(cts_status == 0);
    uart_write_byte(UART_5, *p_data++);	
	}
	while(cts_status1 == 0);
}

void my_usart5_init(void)
{
		uart_init(UART_5, 9600, UART5_TX_C12, UART5_RX_D2);	
	  uart_rx_interrupt(UART_5, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
	  interrupt_set_priority(UART5_IRQn, 2);  
}

void uart_rx_interrupt_handler (void)
{
	unsigned char ucTemp;
		ucTemp = uart_read_byte(UART_5);
		WitSerialDataIn(ucTemp);
}
//------------------------------------------------------------------------------------//
void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;			
		}
	}

}
static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				printf("\r\nSet Baud Error\r\n");
			else 
				uart_init(UART_5, 115200, UART5_TX_C12, UART5_RX_D2);	
	
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else 
				uart_init(UART_5, 9600, UART5_TX_C12, UART5_RX_D2);													
			break;
		case 'R':	
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'r':	
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'C':	
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'c':	
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'h':
			ShowHelp();
			break;
	}
	s_cCmd = 0xff;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	uart_write_buffer(UART_5, p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
	system_delay_ms(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		uart_init(UART_5, c_uiBaud[i], UART5_TX_C12, UART5_RX_D2);	
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);		
			system_delay_ms(100);			
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);	
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

void EXTI4_IRQHandler (void)
{
	 if(gpio_get_level(D4)==0)
	 {
		 if(gpio_get_level(D7)==0)
	   count++;
	 }	 
   else
	 {
		 if(gpio_get_level(D7)==0)
	   count--;
	 }
    EXTI_ClearLineStatus(EXTI, EXTI_LINE_4);                                    // 清除 line4 触发标志
}

void EXTI9_5_IRQHandler (void)
{
    if(EXTI_LINE_5 & EXTI_GetLineStatus(EXTI))                                  // line5 触发
    {
        // 此处编写用户代码 (A5/B5..G5) 引脚触发
	   if(gpio_get_level(D5)==0)
	    {
		    if(gpio_get_level(D14)==0)
	      count1++;
	    }	 
     else
	    {
		    if(gpio_get_level(D14)==0)
	      count1--;
	    }
        // 此处编写用户代码 (A5/B5..G5) 引脚触发
        EXTI_ClearLineStatus(EXTI, EXTI_LINE_5);                                // 清除 line5 触发标志
    }
    if(EXTI_LINE_6 & EXTI_GetLineStatus(EXTI))                                  // line6 触发
    {
        // 此处编写用户代码 (A6/B6..G6) 引脚触发

        // 此处编写用户代码 (A6/B6..G6) 引脚触发
        EXTI_ClearLineStatus(EXTI, EXTI_LINE_6);                                // 清除 line6 触发标志
    }
    if(EXTI_LINE_7 & EXTI_GetLineStatus(EXTI))                                  // line7 触发
    {
        // 此处编写用户代码 (A7/B7..G7) 引脚触发

        // 此处编写用户代码 (A7/B7..G7) 引脚触发
        EXTI_ClearLineStatus(EXTI, EXTI_LINE_7);                                // 清除 line7 触发标志
    }
    if(EXTI_LINE_8 & EXTI_GetLineStatus(EXTI))                                  // line8 触发
    {
        // -----------------* 摄像头 VSY 场中断 预置中断处理函数 *-----------------
        camera_vsync_handler();
        // -----------------* 摄像头 VSY 场中断 预置中断处理函数 *-----------------
        // 此处编写用户代码 (A8/B8..G8) 引脚触发

        // 此处编写用户代码 (A8/B8..G8) 引脚触发
        EXTI_ClearLineStatus(EXTI, EXTI_LINE_8);                                // 清除 line8 触发标志
    }
    if(EXTI_LINE_9 & EXTI_GetLineStatus(EXTI))                                  // line9 触发
    {
        // 此处编写用户代码 (A9/B9..G9) 引脚触发

        // 此处编写用户代码 (A9/B9..G9) 引脚触发
        EXTI_ClearLineStatus(EXTI, EXTI_LINE_9);                                // 清除 line9 触发标志
    }
}

void avoid_barrier_forward()
{
	pwm_set_duty(PWM_CH1, 14); 	
	length=Hcsr04GetLength();
	length=Hcsr04GetLength();
	length=Hcsr04GetLength();
	if(length > 25)
	{
		forward(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));//向前走
	}
	if(length < 25)
	{
		stop_move(0,0,0,0);
	  pwm_set_duty(PWM_CH1, 10);  //向左转30°
	  system_delay_ms(1000);
		length=Hcsr04GetLength();
		length=Hcsr04GetLength();
		length=Hcsr04GetLength();
		if(length > 25)    
		{
		  left_turn(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));
		  system_delay_ms(10);			
		}
		else   //如果左转后前方还有障碍物
		{
		stop_move(0,0,0,0);
	  pwm_set_duty(PWM_CH1, 19);    //向右转30°
	  system_delay_ms(1000);	
		length=Hcsr04GetLength();
	  length=Hcsr04GetLength();
	  length=Hcsr04GetLength();
		if(length > 25)
		 {
			 Right_turn(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));
		   system_delay_ms(10);			
		 }
    else
		 {
			 backward(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));
			 system_delay_ms(1000);
			 Right_turn(pid1(setspeed1),pid2(setspeed2),pid3(setspeed3),pid4(setspeed4));
			 system_delay_ms(10);
		 }			
		}
	}
}
