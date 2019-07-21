#ifndef __IMU_H
#define __IMU_H

#include "stm32f10x.h"


#define Report 1


typedef struct
{
	float Pitch;	//������
	float Roll;		//�����
	float Yaw;		//ƫ����
}AHRS_DATA;


extern AHRS_DATA	AHRS_Data;

void IMUupdate(float ax, float ay, float az,float gx, float gy, float gz);


void IMU_Init(void);
void Get_IMU_Data(void);


#endif


