#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

typedef struct{

	struct{
		float Kp;
		float Ki;
		float Kd;
		float KKp;
		float KKi;
		float KKd;
		float OOut;
	}Pitch;
	
	struct{
		float Kp;
		float Ki;
		float Kd;
		float KKp;
		float KKi;
		float KKd;
		float OOut;
	}Yaw;	
}__PID_Parameters;

extern __PID_Parameters  PID_Parameters;

//PID用户调用函数
void PitchCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output);
void YawCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output);


#endif



