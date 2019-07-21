/**
  ****************************(C) COPYRIGHT 2019 LUT_EDA****************************
  * @file       main.c/h
  * @brief      功能模块程序
  *            
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     July-12-2019     Allen              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 LUT_EDA*****************************/
  
#include "stm32f10x.h"
#include "sys.h"
#include "TIM_Base.h" 
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "oled_IIC.h"





volatile u32 time; // ms 计时变量
volatile u8 key_num;
void OLED_Start(void);//开机画面;
int main(void)
{
	DelayInit();
	NVIC_Configuration();//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	TIM2_NVIC_Configuration(); /* TIM2 定时配置 */
    TIM2_Configuration();
	uart_init(115200);	 //串口初始化为115200
	printf("*************串口测试正常!*************\r\n");
	LED_GPIO_Config();
	Key_GPIO_Config();
	Keys_Init();
	I2C_Configuration();
	OLED_Init();
	OLED_Start();//开机画面
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);	/* TIM2 重新开时钟，开始计时 */
	while(1)
	{
//		if(time<500)
//		{
//			LED1(ON);
//		}
//		if(time>500)
//		{
//			LED1(OFF);
//			if(time>1000)
//				time=0;
//		}
		//PCout(13)=!PCout(13);

		
//		if( Key_Scan(GPIOC,GPIO_Pin_14,1) == KEY_ON  )
//		{
//			/*LED1反转*/
//			LED1_TOGGLE;
//		}
		OLED_Print_Num1(32, 7,key_num);		
		
	}//end of while
}


void OLED_Start(void)//开机画面
{
	int16_t i;
	extern const unsigned char BMP1[];
	OLED_DrawBMP(0,0,128,8,(unsigned char *)BMP1);//测试BMP位图显示
	DelayS(1);
	OLED_CLS();//清屏
	for(i=0;i<6;i++)
	{
		OLED_ShowCN(16+i*16,0,i);//测试显示中文
	}
	DelayMs(500);
	OLED_ShowStr(32,3,"Hello",2);				//测试8*16字符
	DelayMs(500);
	OLED_ShowStr(0,7,"Welcome to LUT_EDA !",1);//测试6*8字符
	DelayS(2);
	OLED_CLS();//清屏
}

/***************************************end of file***********************************/
