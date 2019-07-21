/**
  ****************************(C) COPYRIGHT 2019 LUT_EDA****************************
  * @file       main.c/h
  * @brief      ����ģ�����
  *            
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     July-12-2019     Allen              1. ���
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





volatile u32 time; // ms ��ʱ����
volatile u8 key_num;
void OLED_Start(void);//��������;
int main(void)
{
	DelayInit();
	NVIC_Configuration();//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	TIM2_NVIC_Configuration(); /* TIM2 ��ʱ���� */
    TIM2_Configuration();
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	printf("*************���ڲ�������!*************\r\n");
	LED_GPIO_Config();
	Key_GPIO_Config();
	Keys_Init();
	I2C_Configuration();
	OLED_Init();
	OLED_Start();//��������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);	/* TIM2 ���¿�ʱ�ӣ���ʼ��ʱ */
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
//			/*LED1��ת*/
//			LED1_TOGGLE;
//		}
		OLED_Print_Num1(32, 7,key_num);		
		
	}//end of while
}


void OLED_Start(void)//��������
{
	int16_t i;
	extern const unsigned char BMP1[];
	OLED_DrawBMP(0,0,128,8,(unsigned char *)BMP1);//����BMPλͼ��ʾ
	DelayS(1);
	OLED_CLS();//����
	for(i=0;i<6;i++)
	{
		OLED_ShowCN(16+i*16,0,i);//������ʾ����
	}
	DelayMs(500);
	OLED_ShowStr(32,3,"Hello",2);				//����8*16�ַ�
	DelayMs(500);
	OLED_ShowStr(0,7,"Welcome to LUT_EDA !",1);//����6*8�ַ�
	DelayS(2);
	OLED_CLS();//����
}

/***************************************end of file***********************************/
