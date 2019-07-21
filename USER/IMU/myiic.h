#ifndef __MYIIC_H
#define __MYIIC_H


#include "sys.h"

#define USE_MPU6050 1	//ʹ��MPU6050
#define USE_MPU9250 0	//ʹ��MPU9250

//IO��������
#define SDA_IN()  {GPIOB->CRL&=~(3<<(11*2));GPIOB->CRL|=0<<11*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->CRL&=~(3<<(11*2));GPIOB->CRL|=1<<11 *2;} //PB9���ģʽ
//IO��������	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA   PBin(11)  //����SDA 



//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�



void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  




#endif







