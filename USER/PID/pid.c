#include "pid.h"
#include "rc.h"

#if 1

__PID_Parameters  PID_Parameters;

static float map(long x, long in_min, long in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}    

///************************外环变量********************************/
float PitchOutErr_Now=0,PitchOutErr_Last=0;//Pitch轴当前角度误差,Pitch轴上次角度误差
float PitchOutErr_Integral=0;//外环误差积分
float PitchOutPID_P=0,PitchOutPID_I=0,PitchOutPID_D=0;//外环PID_P项,外环PID_I项,外环PID_D项
float PitchOutPID_Out=0;//外环PID输出
float PitchOut_Diff=0;//外环角度微分

/************************内环变量********************************/
float PitchInsdErr_Now=0,PitchInsdErr_Last=0;//Pitch轴当前角速度误差,Pitch轴上次角速度误差
float PitchInsdErr_Integral=0;//内环误差积分
float PitchInsdPID_P=0,PitchInsdPID_I=0,PitchInsdPID_D=0;//内环PID_P项,内环PID_I项,内环PID_D项
float PitchInsd_Diff=0;//内环角速度微分
//	
//***************Pitch串级PID控制器***********************//
//Out_Target		外环目标角度--用户给定
//Out_Measure		外环实际角度--陀螺仪解算值或编码器测量值
//Inside_Measure	内环实际速度--陀螺仪角速度值
//Inside_Output		唯一输出值--发送到电机
//返回值:无
void PitchCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output)
{
	/**************外环PI*************/
	PitchOutErr_Now=Out_Measure-Out_Target;//当前角度误差
	PitchOutPID_P=PID_Parameters.Pitch.Kp*PitchOutErr_Now;//外环PID_P项
	PitchOutErr_Now+=PitchOutErr_Now;		//当前角度误差积分
	//当前角度误差积分限幅
	if(PitchOutErr_Integral>28000)		PitchOutErr_Integral=28000;
	else if(PitchOutErr_Integral<-28000)	PitchOutErr_Integral=-28000;
	
	PitchOutPID_I=PID_Parameters.Pitch.Ki*PitchOutErr_Integral;//外环PID_I项
	PitchOut_Diff=PitchOutErr_Now-PitchOutErr_Last;//角度微分
	PitchOutErr_Last=PitchOutErr_Now;//更新上次角度偏差
	PitchInsdPID_D=PID_Parameters.Pitch.Kd*PitchOut_Diff;//外环PID_D项
	PitchOutPID_Out=PitchOutPID_P+PitchOutPID_I+PitchOutPID_D;//外环PID输出
	
	/**************内环PID*************/
	PitchInsdErr_Now=PitchOutPID_Out-Inside_Measure;//当前角速度误差	
//	PitchInsdErr_Now=map(RC_Data.RC.ch3,364,1684,-200,200)-Inside_Measure;//当前角速度误差	//速度环调整
	
	PitchInsdPID_P=PID_Parameters.Pitch.KKp*PitchInsdErr_Now;//内环PID_P项
	PitchInsdErr_Integral+=PitchInsdErr_Now;//当前角速度误差积分
	//当前角度误差积分限幅
	if(PitchInsdErr_Integral>28000)			PitchInsdErr_Integral=28000;
	else if(PitchInsdErr_Integral<-28000)	PitchInsdErr_Integral=-28000;
	
	PitchInsdPID_I=PID_Parameters.Pitch.KKi*PitchInsdErr_Integral;//内环PID_I项
	PitchInsd_Diff=PitchInsdErr_Now-PitchInsdErr_Last;//当前角速度微分
	PitchInsdErr_Last=PitchInsdErr_Now;//更新上次角速度误差
	PitchInsdPID_D=PID_Parameters.Pitch.KKd*PitchInsd_Diff;//内环PID_D项
	
	*Inside_Output=PitchInsdPID_P+PitchInsdPID_I+PitchInsdPID_D;
}


/************************外环变量********************************/
float YawOutErr_Now=0,YawOutErr_Last=0;//Pitch轴当前角度误差,Pitch轴上次角度误差
float YawOutErr_Integral=0;//外环误差积分
float YawOutPID_P=0,YawOutPID_I=0,YawOutPID_D=0;//外环PID_P项,外环PID_I项,外环PID_D项
float YawOutPID_Out=0;//外环PID输出
float YawOut_Diff=0;//外环角度微分

/************************内环变量********************************/
float YawInsdErr_Now=0,YawInsdErr_Last=0;//Pitch轴当前角速度误差,Pitch轴上次角速度误差
float YawInsdErr_Integral=0;//内环误差积分
float YawInsdPID_P=0,YawInsdPID_I=0,YawInsdPID_D=0;//内环PID_P项,内环PID_I项,内环PID_D项
float YawInsd_Diff=0;//内环角速度微分
	
//***************Yaw串级PID控制器***********************//
//Out_Target		外环目标角度--用户给定
//Out_Measure		外环实际角度--陀螺仪解算值或编码器测量值
//Inside_Measure	内环实际速度--陀螺仪角速度值
//Inside_Output		唯一输出值--发送到电机
//返回值:无
void YawCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output)
{
	/**************外环PI*************/
	YawOutErr_Now=Out_Measure-Out_Target;//当前角度误差
	YawOutPID_P=PID_Parameters.Yaw.Kp*YawOutErr_Now;//外环PID_P项
	YawOutErr_Integral+=YawOutErr_Now;		//当前角度误差积分
	//当前角度误差积分限幅
	if(YawOutErr_Integral>28000)		YawOutErr_Integral=28000;
	else if(YawOutErr_Integral<-28000)	YawOutErr_Integral=-28000;
	
	YawOutPID_I=PID_Parameters.Yaw.Ki*YawOutErr_Integral;//外环PID_I项
	YawOut_Diff=YawOutErr_Now-YawOutErr_Last;//角度微分
	YawOutErr_Last=YawOutErr_Now;//更新上次角度偏差
	YawInsdPID_D=PID_Parameters.Yaw.Kd*YawOut_Diff;//外环PID_D项
	YawOutPID_Out=YawOutPID_P+YawOutPID_I+YawOutPID_D;//外环PID输出
	
	/**************内环PID*************/
	YawInsdErr_Now=YawOutPID_Out-Inside_Measure;//当前角速度误差	
//	YawInsdErr_Now=map(RC_Data.RC.ch0,364,1684,-200,200)-Inside_Measure;//当前角速度误差	//调试速度环用
	
	YawInsdPID_P=PID_Parameters.Yaw.KKp*YawInsdErr_Now;//内环PID_P项
	YawInsdErr_Integral+=YawInsdErr_Now;//当前角速度误差积分
	//当前角度误差积分限幅
	if(YawInsdErr_Integral>28000)			YawInsdErr_Integral=28000;
	else if(YawInsdErr_Integral<-28000)		YawInsdErr_Integral=-28000;
	
	YawInsdPID_I=PID_Parameters.Yaw.KKi*YawInsdErr_Integral;//内环PID_I项
	YawInsd_Diff=YawInsdErr_Now-YawInsdErr_Last;//当前角速度微分
	YawInsdErr_Last=YawInsdErr_Now;//更新上次角速度误差
	YawInsdPID_D=PID_Parameters.Yaw.KKd*YawInsd_Diff;//内环PID_D项
	
	*Inside_Output=YawInsdPID_P+YawInsdPID_I+YawInsdPID_D;
}


#else

//外环参数
float Pitch_Kp=10;//10;
float Pitch_Ki=0.02;//0.1;
float Pitch_Kd=16;//12;
//内环参数
float Pitch_KKp=8.0;
float Pitch_KKi=0;
float Pitch_KKd=20;

float Pitch_i;          //积分项
float Pitch_old;        //角度保存
float Pitch_d;          //微分项
float Pitch_shell_out;//外环总输出

float Gyro_radian_old_y;//陀螺仪保存
float pitch_core_kp_out,pitch_core_kd_out;//内环单项输出
float Pitch_core_out;//内环总输出

//***************Pitch串级PID控制器***********************//
//Out_Target		外环目标角度--用户给定
//Out_Measure		外环实际角度--陀螺仪解算值或编码器测量值
//Inside_Measure	内环实际速度--陀螺仪角速度值
//Inside_Output		唯一输出值--发送到电机
//返回值:无
/*********************************/
void PitchCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output)
{
	////////////////////////外环角度环(PID)///////////////////////////////
	Pitch_i+=(Out_Measure-Out_Target);
	//-------------Pitch积分限幅----------------//
	if(Pitch_i>3000) Pitch_i=3000;
	else if(Pitch_i<-3000) Pitch_i=-3000;
	//-------------Pitch微分--------------------//
	Pitch_d=Out_Measure-Pitch_old;
	//-------------Pitch  PID-------------------//
	Pitch_shell_out = Pitch_Kp*(Out_Measure-Out_Target) + Pitch_Ki*Pitch_i + Pitch_Kd*Pitch_d;
	//角度保存
	Pitch_old=Out_Measure;
	/*********************************************************/
	////////////////////////内环角速度环(PD)///////////////////////////////
	pitch_core_kp_out = Pitch_KKp * (Pitch_shell_out + Inside_Measure);
	pitch_core_kd_out = Pitch_KKd * (Inside_Measure  - Gyro_radian_old_y);

	Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
	Gyro_radian_old_y = Inside_Measure;//储存历史值
	
	*Inside_Output=Pitch_core_out;
}

#endif

