#include "dev_mpu6050.h"
#include "usart.h"
#include "led.h"
//#include "timer.h"
#include "flash.h"
#include "dial.h"

#include <math.h>


GYRO_CALI_DATA		GyroCaliData;
MPU6050_RAW_DATA	MPU6050_Data;
ACCEL_CALI_DATA		AccelCaliData;

uint8_t MPU_HEALTH=0;
uint8_t mpu_buf[14]={0};       //save the data of acc gyro using iic
int16_t MPU6050_FIFO[6]= {0};//10�����ݵ�ƽ��ֵ
int16_t MPU6050_Template;

unsigned char IMU_Sta=0;

//��ʱ����������
static void Delay_ms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=28000;  
      while(i--) ;    
   }
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=IIC_Read_Byte(0);//��ȡ����,����nACK 
    IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU6050_Init(void)
{ 
	//LED_GPIO_Init();
	u8 res;	
	IIC_Init();//��ʼ��IIC����
	
	//Dial_Init();//���뿪�س�ʼ��

	Delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    Delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz

	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
 	}
	else
	{
		printf("\rERROR-->>0x1A Read_ID:0x%x\r\n",res);
		return 1;
	}
	//�����Ƕ�ȡ���ݲ��ȶ���Ԥ�ȶ�ȡ500��
	for(int i=0;i<500;i++)
	{
		MPU6050_GetData();
	}
	
	Gyro_Calibration();
	
	if(CaliAccel_EN)
	{
		printf("�ȴ�У׼���ٶȼ�");
		while(1)
		{
			LED_Sta=LED_STA_AccelGyroOffsetError;
			Accel_Clibration(IMU_Sta);
		}
	}
	
	Get_ACCEL_Offset();//��ȡ���ٶȼ�offsetֵ
	
	return 0;
}

/**********************************************************************************/
/**********************************************************************************/

int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;

void MPU6050_GetData(void)
{
	MPU_Read_Len(MPU_ADDR,MPU6050_DATA_START,14,mpu_buf);  //��MPU_ACCEL_XOUTH_REG��ʼ��ȡ����
	MPU6050_Lastax=(((int16_t)mpu_buf[0]) << 8) | mpu_buf[1];
	MPU6050_Lastay=(((int16_t)mpu_buf[2]) << 8) | mpu_buf[3];
	MPU6050_Lastaz=(((int16_t)mpu_buf[4]) << 8) | mpu_buf[5];
	
	MPU6050_Template=(((int16_t)mpu_buf[6]) << 8) | mpu_buf[7];
	
	MPU6050_Lastgx=(((int16_t)mpu_buf[8]) << 8)  | mpu_buf[9];
	MPU6050_Lastgy=(((int16_t)mpu_buf[10]) << 8) | mpu_buf[11];
	MPU6050_Lastgz=(((int16_t)mpu_buf[12]) << 8) | mpu_buf[13];
}

void MPU6050_Filter(void)
{
	int Sum[6]={0};
	
	for(int i=0;i<10;i++)
	{
		MPU6050_GetData();
		Sum[0]+=MPU6050_Lastax;
		Sum[1]+=MPU6050_Lastay;
		Sum[2]+=MPU6050_Lastaz;
		Sum[3]+=MPU6050_Lastgx;
		Sum[4]+=MPU6050_Lastgy;
		Sum[5]+=MPU6050_Lastgz-((MPU6050_Template/340.0+36.53)*0.5-13);
	}
		MPU6050_FIFO[0]=Sum[0]/10;
		MPU6050_FIFO[1]=Sum[1]/10;
		MPU6050_FIFO[2]=Sum[2]/10;
		MPU6050_FIFO[3]=Sum[3]/10;
		MPU6050_FIFO[4]=Sum[4]/10;
		MPU6050_FIFO[5]=Sum[5]/10;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
*��������:	    ��ȡ MPU6050�ĵ�ǰ����ֵ
*******************************************************************************/
void MPU6050_getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) 
{
//	MPU6050_GetData();
	MPU6050_Filter();
	
//	*ax=(MPU6050_FIFO[0] - 808	)*9.8/16384;
//	*ay=(MPU6050_FIFO[1] + 234.5)*9.8/16384;
//	*az=(MPU6050_FIFO[2] - 459.5)*9.8/16384;
	*ax=(MPU6050_FIFO[0] - AccelCaliData.CaliAccelOffsetX)*9.8f/16384;
	*ay=(MPU6050_FIFO[1] - AccelCaliData.CaliAccelOffsetY)*9.8f/16384;
	*az=(MPU6050_FIFO[2] - AccelCaliData.CaliAccelOffsetZ)*9.8f/16384;	
	*gx=(MPU6050_FIFO[3] - GyroCaliData.GyroXOffset)/16.4f;
	*gy=(MPU6050_FIFO[4] - GyroCaliData.GyroYOffset)/16.4f;
	*gz=(MPU6050_FIFO[5] - GyroCaliData.GyroZOffset)/16.4f;
}

void Gyro_Calibration(void)
{
	int CaliSum[3]={0};
	
	MPU6050_Lastgx=0;
	MPU6050_Lastgy=0;
	MPU6050_Lastgz=0;
	
	for(int i=0;i<5000;i++)
	{
		MPU6050_GetData();
				
		CaliSum[0]+=MPU6050_Lastgx;
		CaliSum[1]+=MPU6050_Lastgy;
		CaliSum[2]+=MPU6050_Lastgz;
		
	}

	GyroCaliData.GyroXOffset=CaliSum[0]/5000;
	GyroCaliData.GyroYOffset=CaliSum[1]/5000;
	GyroCaliData.GyroZOffset=CaliSum[2]/5000;
}


#if 1

/******** У�����ٶȼ���Ҫ��Offsetֵ ********/
//AccelXZOffset-----[ 17156]-16384=772
//AccelXFOffset-----[-15540]+16384=844
//��ֵ=808
//AccelYZOffset-----[ 16148]-16384=-236
//AccelYFOffset-----[-16617]+16384=-233
//��ֵ=-234.5
//AccelZZOffset-----[ 16533]-16384=149
//AccelZFOffset-----[-17452]+16384=-1068
//��ֵ=-459.5

void Accel_Clibration(u8 Accel_x_Cali)
{
	if(Accel_x_Cali==ACCEL_XZ_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastax=0;
		
		printf("У׼���ٶȼ�-->X+");
		
		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=MPU6050_Lastax;
			Delay_ms(1);
		}

		MPU6050_Lastax=0;
		AccelCaliData.AccelXZOffset=CaliSum/5000;
		printf("У׼���X+���ٶ�OffSetֵ-->>%d\r\n",AccelCaliData.AccelXZOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_XZ_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_XF_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastax=0;

		printf("У׼���ٶȼ�-->X-");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=-MPU6050_Lastax;
			Delay_ms(1);
		}

		MPU6050_Lastax=0;
		AccelCaliData.AccelXFOffset=-CaliSum/5000;
		printf("У׼���X-���ٶ�OffSetֵ-->>%d\r\n",AccelCaliData.AccelXFOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_XF_CALI)break;
		}
	}	
	else if(Accel_x_Cali==ACCEL_YZ_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastay=0;

		printf("У׼���ٶȼ�-->Y+");
		
		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=MPU6050_Lastay;
			Delay_ms(1);
		}
		MPU6050_Lastay=0;
		AccelCaliData.AccelYZOffset=CaliSum/5000;
		printf("У׼���Y+���ٶ�OffSetֵ-->>%d\r\n",AccelCaliData.AccelYZOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_YZ_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_YF_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastay=0;

		printf("У׼���ٶȼ�-->Y-");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=-MPU6050_Lastay;
			Delay_ms(1);
		}
		MPU6050_Lastay=0;
		AccelCaliData.AccelYFOffset=-CaliSum/5000;
		printf("У׼���Y-���ٶ�OffSetֵ-->>%d\r\n",AccelCaliData.AccelYFOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_YF_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_ZZ_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastaz=0;

		printf("У׼���ٶȼ�-->Z+");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=MPU6050_Lastaz;
			Delay_ms(1);
		}
		MPU6050_Lastaz=0;
		AccelCaliData.AccelZZOffset=CaliSum/5000;
		printf("У׼���Z+���ٶ�OffSetֵ-->>%d\r\n",AccelCaliData.AccelZZOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_ZZ_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_ZF_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastaz=0;

		printf("У׼���ٶȼ�-->Z-");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=-MPU6050_Lastaz;
			Delay_ms(1);
		}
		MPU6050_Lastaz=0;
		AccelCaliData.AccelZFOffset=-CaliSum/5000;
		printf("У׼���Z-���ٶ�OffSetֵ-->>%d\r\n",AccelCaliData.AccelZFOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_ZF_CALI)break;
		}
	}	
	else if(Accel_x_Cali==0x07)//��У׼�������д��flash����ַ:0x080E0000
	{
		int16_t Temp[3];
		u32 FlashData[3]={0};
		Temp[0]=(AccelCaliData.AccelXFOffset+AccelCaliData.AccelXZOffset)/2;
		Temp[1]=(AccelCaliData.AccelYFOffset+AccelCaliData.AccelYZOffset)/2;
		Temp[2]=(AccelCaliData.AccelZFOffset+AccelCaliData.AccelZZOffset)/2;
		
		FlashData[0]=(u32)Temp[0]<<16|(u32)0xFF;
		FlashData[1]=(u32)Temp[1]<<16|(u32)0xFF;
		FlashData[2]=(u32)Temp[2]<<16|(u32)0xFF;
		
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)==1)
		{
			FLASH_Write(IMU_ACCEL_OFFSET_ADDR,FlashData,3);
			Get_ACCEL_Offset();
			//У��flash����
			if((AccelCaliData.CaliAccelOffsetX==Temp[0])||(AccelCaliData.CaliAccelOffsetY==Temp[1])||(AccelCaliData.CaliAccelOffsetZ==Temp[2]))
			{
				printf("���ٶȼ���У׼;�������ϵ�");
				while(1)
				{
					DN_TIM2;DN_TIM3;
					LED1=0;LED2=0;
				}	
			}			
			else
			{
				printf("���ٶȼ�У׼ʧ�ܣ��������ϵ�У׼");		
				while(1)
				{
					DN_TIM2;DN_TIM3;
					LED1=1;LED2=0;
				}	
			}	
		}

	}	
	else
	{
		Delay_ms(1000);
		printf(".");
	}	
}

#endif





