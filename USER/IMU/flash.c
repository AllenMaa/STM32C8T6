#include "flash.h"
#include "dev_mpu6050.h"
#include "led.h"
 

/****************************************************************************
* ��    ��: ��ȡ��ַAddress��Ӧ��sector���
* ��ڲ�������ַ
* ���ڲ�����sector���
* ˵    ������
* ���÷�������
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
* ��    ��: ��ȡ��������
* ��ڲ�����faddr����ַ
* ���ڲ�������
* ˵    ������
* ���÷�������
****************************************************************************/
u32 FLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
/****************************************************************************
* ��    ��: ����ָ������
* ��ڲ�����SectorNum ������
* ���ڲ�������
* ˵    ������
* ���÷�������
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
* ��    ��: д�볤��Ϊlength��32λ����
* ��ڲ�����WriteAddr����ַ
			NumToWrite�� ���ݳ���
			pBuffer��Ҫд�������ָ��
* ���ڲ�������
* ˵    ������
* ���÷�������
****************************************************************************/
void FLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<ADDR_FLASH_SECTOR_0||WriteAddr%4)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(FLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(Flash_GetSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
} 

/****************************************************************************
* ��    ��: ��ȡ����Ϊlength��32λ����
* ��ڲ�����ReadAddr����ַ
			NumToRead�� ���ݳ���
			pBuffer����ȡ������ָ��
* ���ڲ�������
* ˵    ������
* ���÷�������
****************************************************************************/
void FLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=FLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

/****************************************************************************
* ��    ��: ��ȡ���ٶȼ�Offset����
* ��ڲ�������
* ���ڲ�������
* ˵    ������
* ���÷�������
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




