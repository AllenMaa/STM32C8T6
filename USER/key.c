/**
  ******************************************************************************
  * @file    bsp_key.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ����Ӧ��bsp��ɨ��ģʽ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "key.h" 
#include "usart.h"

/// ����ȷ����ʱ
static void Key_Delay(__IO u32 nCount)
{
	for(; nCount != 0; nCount--);
} 

/**
  * @brief  ���ð����õ���I/O��
  * @param  ��
  * @retval ��
  */
void Key_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*���������˿ڣ�PC����ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief   ����Ƿ��а�������
  * @param   ����Ķ˿ںͶ˿�λ
  *		@arg GPIOx: x�����ǣ�A...G�� 
  *		@arg GPIO_PIN ������GPIO_PIN_x��x������1...16��
  *   @arg Down_state:��������ʱ�ĵ�ƽ��1Ϊ�ߵ�ƽ��0Ϊ�͵�ƽ
  * @retval  ������״̬
  *		@arg KEY_ON:��������
  *		@arg KEY_OFF:����û����
  */
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin,uint8_t Down_state)
{			
	/*����Ƿ��а������� */
	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Down_state ) 
	{	   
		/*��ʱ����*/
		Key_Delay(10000);		
		if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == Down_state )  
		{	 
			/*�ȴ������ͷ� */
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��PORTAʱ��
	
	GPIO_InitStructure.GPIO_Pin  = KEY_L;//PA0 PA1 PA2 PA3
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //���ó��������
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA0 / 1 / 2 / 3
	
	GPIO_InitStructure.GPIO_Pin  = KEY_H;//PA4 PA5 PA6 PA7
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA4 / 5 / 6 / 7
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);	
	GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
		
}


int KeyValue=0;
//����ɨ�躯��
//���ذ���ֵ
//����ֵ��
//0��û���κΰ�������
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

//����������
//���ݰ�������ִֵ������Ӧ�Ķ���
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
				/*�û������Ӧ����*/
		}
		if(KeyValue == 4)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 5)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 6)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 7)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 8)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 9)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 10)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 11)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 12)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 13)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 14)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 15)
		{
				/*�û������Ӧ����*/
		}
		if(KeyValue == 16)
		{
				/*�û������Ӧ����*/
		}
}
/*********************************************END OF FILE**********************/
