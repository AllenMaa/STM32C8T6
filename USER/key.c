/**
  ******************************************************************************
  * @file    bsp_key.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   按键应用bsp（扫描模式）
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "key.h" 
#include "usart.h"

/// 不精确的延时
static void Key_Delay(__IO u32 nCount)
{
	for(; nCount != 0; nCount--);
} 

/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*开启按键端口（PC）的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief   检测是否有按键按下
  * @param   具体的端口和端口位
  *		@arg GPIOx: x可以是（A...G） 
  *		@arg GPIO_PIN 可以是GPIO_PIN_x（x可以是1...16）
  *   @arg Down_state:按键按下时的电平，1为高电平，0为低电平
  * @retval  按键的状态
  *		@arg KEY_ON:按键按下
  *		@arg KEY_OFF:按键没按下
  */
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin,uint8_t Down_state)
{			
	/*检测是否有按键按下 */
	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Down_state ) 
	{	   
		/*延时消抖*/
		Key_Delay(10000);		
		if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Down_state )  
		{	 
			/*等待按键释放 */
			while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Down_state);   
			return 	KEY_ON;	 
		}
		else
			return KEY_OFF;
	}
	else
		return KEY_OFF;
}

void Keys_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能PORTA时钟
	
	GPIO_InitStructure.GPIO_Pin  = KEY_L;//PA0 PA1 PA2 PA3
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //设置成推挽输出
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA0 / 1 / 2 / 3
	
	GPIO_InitStructure.GPIO_Pin  = KEY_H;//PA4 PA5 PA6 PA7
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA4 / 5 / 6 / 7
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
		
}


int KeyValue=0;
//按键扫描函数
//返回按键值
//返回值：
//0，没有任何按键按下
int Keys_Scan(void)
{	
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==SET
	  &&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==SET
	  &&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET
	  &&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET) KeyValue=0;
	 
	 GPIO_SetBits(GPIOA,GPIO_Pin_0); 
	 GPIO_ResetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=1;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==SET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=2;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=3;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==SET) KeyValue=4;
	 
	 GPIO_SetBits(GPIOA,GPIO_Pin_1); 
	 GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3);
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=5;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==SET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=6;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=7;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==SET) KeyValue=8;
	 
	 GPIO_SetBits(GPIOA,GPIO_Pin_2); 
	 GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3);
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=9;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==SET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=10;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=11;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==SET) KeyValue=12;
	 
	 GPIO_SetBits(GPIOA,GPIO_Pin_3); 
	 GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
    
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=13;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==SET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=14;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==SET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==RESET) KeyValue=15;
	 
	 if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==RESET&&
		GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)==RESET&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==SET) KeyValue=16;
	 
	 GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	 GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6| GPIO_Pin_7); 
		    return KeyValue;
}

//按键处理函数
//根据按键返回值执行响相应的动作
void Keys_Action(void)
{
		if(KeyValue == 1)
		{
			printf("press 1\r\n");
				//LED0 = !LED0;
		}
		if(KeyValue == 2)
		{
				printf("press 2\r\n");
		}
		if(KeyValue == 3)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 4)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 5)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 6)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 7)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 8)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 9)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 10)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 11)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 12)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 13)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 14)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 15)
		{
				/*用户添加相应程序*/
		}
		if(KeyValue == 16)
		{
				/*用户添加相应程序*/
		}
}
/*********************************************END OF FILE**********************/
