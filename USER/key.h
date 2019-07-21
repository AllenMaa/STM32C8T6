#ifndef __KEY_H
#define	__KEY_H

#include "stm32f10x.h"
 /*******
 *�������±���
 KEY_ON 0
 KEY_OFF 1
 ********/
#define KEY_ON	0
#define KEY_OFF	1

#define KEY_L GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
#define KEY_H GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
/**********��������***********/
void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin,uint8_t Down_state);
/**********�������**********/
void Keys_Init(void);//IO�����ʼ��
int Keys_Scan(void);
void Keys_Action(void);//����ִ�к���

#endif /* __LED_H */

