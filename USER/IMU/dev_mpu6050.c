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
int16_t MPU6050_FIFO[6]= {0};//10次数据的平均值
int16_t MPU6050_Template;

unsigned char IMU_Sta=0;

//延时函数，毫秒
static void Delay_ms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=28000;  
      while(i--) ;    
   }
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU6050_Init(void)
{ 
	//LED_GPIO_Init();
	u8 res;	
	IIC_Init();//初始化IIC总线
	
	//Dial_Init();//拨码开关初始化

	Delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    Delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz

	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
 	}
	else
	{
		printf("\rERROR-->>0x1A Read_ID:0x%x\r\n",res);
		return 1;
	}
	//陀螺仪读取数据不稳定，预先读取500次
	for(int i=0;i<500;i++)
	{
		MPU6050_GetData();
	}
	
	Gyro_Calibration();
	
	if(CaliAccel_EN)
	{
		printf("等待校准加速度计");
		while(1)
		{
			LED_Sta=LED_STA_AccelGyroOffsetError;
			Accel_Clibration(IMU_Sta);
		}
	}
	
	Get_ACCEL_Offset();//获取加速度计offset值
	
	return 0;
}

/**********************************************************************************/
/**********************************************************************************/

int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;

void MPU6050_GetData(void)
{
	MPU_Read_Len(MPU_ADDR,MPU6050_DATA_START,14,mpu_buf);  //从MPU_ACCEL_XOUTH_REG开始读取数据
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

/**************************实现函数********************************************
*函数原型:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
*功　　能:	    读取 MPU6050的当前测量值
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

/******** 校正加速度计需要的Offset值 ********/
//AccelXZOffset-----[ 17156]-16384=772
//AccelXFOffset-----[-15540]+16384=844
//均值=808
//AccelYZOffset-----[ 16148]-16384=-236
//AccelYFOffset-----[-16617]+16384=-233
//均值=-234.5
//AccelZZOffset-----[ 16533]-16384=149
//AccelZFOffset-----[-17452]+16384=-1068
//均值=-459.5

void Accel_Clibration(u8 Accel_x_Cali)
{
	if(Accel_x_Cali==ACCEL_XZ_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastax=0;
		
		printf("校准加速度计-->X+");
		
		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=MPU6050_Lastax;
			Delay_ms(1);
		}

		MPU6050_Lastax=0;
		AccelCaliData.AccelXZOffset=CaliSum/5000;
		printf("校准后的X+加速度OffSet值-->>%d\r\n",AccelCaliData.AccelXZOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_XZ_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_XF_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastax=0;

		printf("校准加速度计-->X-");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=-MPU6050_Lastax;
			Delay_ms(1);
		}

		MPU6050_Lastax=0;
		AccelCaliData.AccelXFOffset=-CaliSum/5000;
		printf("校准后的X-加速度OffSet值-->>%d\r\n",AccelCaliData.AccelXFOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_XF_CALI)break;
		}
	}	
	else if(Accel_x_Cali==ACCEL_YZ_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastay=0;

		printf("校准加速度计-->Y+");
		
		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=MPU6050_Lastay;
			Delay_ms(1);
		}
		MPU6050_Lastay=0;
		AccelCaliData.AccelYZOffset=CaliSum/5000;
		printf("校准后的Y+加速度OffSet值-->>%d\r\n",AccelCaliData.AccelYZOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_YZ_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_YF_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastay=0;

		printf("校准加速度计-->Y-");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=-MPU6050_Lastay;
			Delay_ms(1);
		}
		MPU6050_Lastay=0;
		AccelCaliData.AccelYFOffset=-CaliSum/5000;
		printf("校准后的Y-加速度OffSet值-->>%d\r\n",AccelCaliData.AccelYFOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_YF_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_ZZ_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastaz=0;

		printf("校准加速度计-->Z+");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=MPU6050_Lastaz;
			Delay_ms(1);
		}
		MPU6050_Lastaz=0;
		AccelCaliData.AccelZZOffset=CaliSum/5000;
		printf("校准后的Z+加速度OffSet值-->>%d\r\n",AccelCaliData.AccelZZOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_ZZ_CALI)break;
		}
	}
	else if(Accel_x_Cali==ACCEL_ZF_CALI)
	{
		int CaliSum=0;
		MPU6050_Lastaz=0;

		printf("校准加速度计-->Z-");

		for(int i=0;i<5000;i++)
		{
			MPU6050_GetData();
			CaliSum+=-MPU6050_Lastaz;
			Delay_ms(1);
		}
		MPU6050_Lastaz=0;
		AccelCaliData.AccelZFOffset=-CaliSum/5000;
		printf("校准后的Z-加速度OffSet值-->>%d\r\n",AccelCaliData.AccelZFOffset);
		while(1)
		{
			if(IMU_Sta!=ACCEL_ZF_CALI)break;
		}
	}	
	else if(Accel_x_Cali==0x07)//将校准后的数据写入flash，地址:0x080E0000
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
			//校验flash数据
			if((AccelCaliData.CaliAccelOffsetX==Temp[0])||(AccelCaliData.CaliAccelOffsetY==Temp[1])||(AccelCaliData.CaliAccelOffsetZ==Temp[2]))
			{
				printf("加速度计已校准;请重新上电");
				while(1)
				{
					DN_TIM2;DN_TIM3;
					LED1=0;LED2=0;
				}	
			}			
			else
			{
				printf("加速度计校准失败，请重新上电校准");		
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





