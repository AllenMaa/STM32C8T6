#include "dial.h"
#include "dev_mpu6050.h"

//延时函数，毫秒
static void Delay_ms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  
      while(i--) ;    
   }
}

u8	CaliAccel_EN=0;

void Dial_Init(void)
{
	GPIO_InitTypeDef	gpio;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	gpio.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	gpio.GPIO_Mode=GPIO_Mode_IN;
	gpio.GPIO_OType=GPIO_OType_PP;
	gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC,&gpio);
	
	EXTI_GPIO_Init();
	
	CaliAccel_EN=(GPIOC->IDR>>3)&0x01;
}

void EXTI_GPIO_Init(void)
{
	NVIC_InitTypeDef	nvic;
	EXTI_InitTypeDef	exti;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟	

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0); 	
	
	nvic.NVIC_IRQChannel=EXTI0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority=0;
	nvic.NVIC_IRQChannelSubPriority=0;
	nvic.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&nvic);

	exti.EXTI_Line=EXTI_Line0;
	exti.EXTI_Mode=EXTI_Mode_Interrupt;
	exti.EXTI_Trigger=EXTI_Trigger_Rising;
	exti.EXTI_LineCmd=ENABLE;
	EXTI_Init(&exti);
}


void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
	{	
		Delay_ms(200);
		if(GPIOC->IDR&0x01)
			IMU_Sta++;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}



