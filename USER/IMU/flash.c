#include "flash.h"
#include "dev_mpu6050.h"
#include "led.h"
 

/****************************************************************************
* 功    能: 获取地址Address对应的sector编号
* 入口参数：地址
* 出口参数：sector编号
* 说    明：无
* 调用方法：无
****************************************************************************/
uint16_t Flash_GetSector(uint32_t Address)
{
	if(Address<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(Address<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(Address<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(Address<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(Address<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(Address<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(Address<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(Address<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(Address<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(Address<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(Address<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
 
/****************************************************************************
* 功    能: 读取半字数据
* 入口参数：faddr：地址
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/
u32 FLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
/****************************************************************************
* 功    能: 擦除指定扇区
* 入口参数：SectorNum 扇区号
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/
void Flash_EraseSector(uint16_t SectorNum)
{
	FLASH_Unlock(); 
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	if (FLASH_EraseSector(SectorNum, VoltageRange_3) != FLASH_COMPLETE) while (1);
	FLASH_Lock(); 
}
 
/****************************************************************************
* 功    能: 写入长度为length的32位数据
* 入口参数：WriteAddr：地址
			NumToWrite： 数据长度
			pBuffer：要写入的数据指针
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/
void FLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<ADDR_FLASH_SECTOR_0||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(FLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(Flash_GetSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 

/****************************************************************************
* 功    能: 读取长度为length的32位数据
* 入口参数：ReadAddr：地址
			NumToRead： 数据长度
			pBuffer：读取的数据指针
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/
void FLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=FLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}

/****************************************************************************
* 功    能: 读取加速度计Offset数据
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无
****************************************************************************/
void Get_ACCEL_Offset(void)
{
	u32 temp[3];
	FLASH_Read(IMU_ACCEL_OFFSET_ADDR,temp,3);
	
	AccelCaliData.CaliAccelOffsetX=(int16_t)(temp[0]>>16);
	AccelCaliData.CaliAccelOffsetY=(int16_t)(temp[1]>>16);
	AccelCaliData.CaliAccelOffsetZ=(int16_t)(temp[2]>>16);
	
	if((AccelCaliData.CaliAccelOffsetX==0)||(AccelCaliData.CaliAccelOffsetY==0)||(AccelCaliData.CaliAccelOffsetZ==0))
	{
		LED_Sta=LED_STA_AccelGyroOffsetError;
		IMU_Sta=0x07;
		while(1);
	}
	
}




