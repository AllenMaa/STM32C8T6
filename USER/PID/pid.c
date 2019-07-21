#include "pid.h"
#include "rc.h"

#if 1

__PID_Parameters  PID_Parameters;

static float map(long x, long in_min, long in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}    

///************************�⻷����********************************/
float PitchOutErr_Now=0,PitchOutErr_Last=0;//Pitch�ᵱǰ�Ƕ����,Pitch���ϴνǶ����
float PitchOutErr_Integral=0;//�⻷������
float PitchOutPID_P=0,PitchOutPID_I=0,PitchOutPID_D=0;//�⻷PID_P��,�⻷PID_I��,�⻷PID_D��
float PitchOutPID_Out=0;//�⻷PID���
float PitchOut_Diff=0;//�⻷�Ƕ�΢��

/************************�ڻ�����********************************/
float PitchInsdErr_Now=0,PitchInsdErr_Last=0;//Pitch�ᵱǰ���ٶ����,Pitch���ϴν��ٶ����
float PitchInsdErr_Integral=0;//�ڻ�������
float PitchInsdPID_P=0,PitchInsdPID_I=0,PitchInsdPID_D=0;//�ڻ�PID_P��,�ڻ�PID_I��,�ڻ�PID_D��
float PitchInsd_Diff=0;//�ڻ����ٶ�΢��
//	
//***************Pitch����PID������***********************//
//Out_Target		�⻷Ŀ��Ƕ�--�û�����
//Out_Measure		�⻷ʵ�ʽǶ�--�����ǽ���ֵ�����������ֵ
//Inside_Measure	�ڻ�ʵ���ٶ�--�����ǽ��ٶ�ֵ
//Inside_Output		Ψһ���ֵ--���͵����
//����ֵ:��
void PitchCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output)
{
	/**************�⻷PI*************/
	PitchOutErr_Now=Out_Measure-Out_Target;//��ǰ�Ƕ����
	PitchOutPID_P=PID_Parameters.Pitch.Kp*PitchOutErr_Now;//�⻷PID_P��
	PitchOutErr_Now+=PitchOutErr_Now;		//��ǰ�Ƕ�������
	//��ǰ�Ƕ��������޷�
	if(PitchOutErr_Integral>28000)		PitchOutErr_Integral=28000;
	else if(PitchOutErr_Integral<-28000)	PitchOutErr_Integral=-28000;
	
	PitchOutPID_I=PID_Parameters.Pitch.Ki*PitchOutErr_Integral;//�⻷PID_I��
	PitchOut_Diff=PitchOutErr_Now-PitchOutErr_Last;//�Ƕ�΢��
	PitchOutErr_Last=PitchOutErr_Now;//�����ϴνǶ�ƫ��
	PitchInsdPID_D=PID_Parameters.Pitch.Kd*PitchOut_Diff;//�⻷PID_D��
	PitchOutPID_Out=PitchOutPID_P+PitchOutPID_I+PitchOutPID_D;//�⻷PID���
	
	/**************�ڻ�PID*************/
	PitchInsdErr_Now=PitchOutPID_Out-Inside_Measure;//��ǰ���ٶ����	
//	PitchInsdErr_Now=map(RC_Data.RC.ch3,364,1684,-200,200)-Inside_Measure;//��ǰ���ٶ����	//�ٶȻ�����
	
	PitchInsdPID_P=PID_Parameters.Pitch.KKp*PitchInsdErr_Now;//�ڻ�PID_P��
	PitchInsdErr_Integral+=PitchInsdErr_Now;//��ǰ���ٶ�������
	//��ǰ�Ƕ��������޷�
	if(PitchInsdErr_Integral>28000)			PitchInsdErr_Integral=28000;
	else if(PitchInsdErr_Integral<-28000)	PitchInsdErr_Integral=-28000;
	
	PitchInsdPID_I=PID_Parameters.Pitch.KKi*PitchInsdErr_Integral;//�ڻ�PID_I��
	PitchInsd_Diff=PitchInsdErr_Now-PitchInsdErr_Last;//��ǰ���ٶ�΢��
	PitchInsdErr_Last=PitchInsdErr_Now;//�����ϴν��ٶ����
	PitchInsdPID_D=PID_Parameters.Pitch.KKd*PitchInsd_Diff;//�ڻ�PID_D��
	
	*Inside_Output=PitchInsdPID_P+PitchInsdPID_I+PitchInsdPID_D;
}


/************************�⻷����********************************/
float YawOutErr_Now=0,YawOutErr_Last=0;//Pitch�ᵱǰ�Ƕ����,Pitch���ϴνǶ����
float YawOutErr_Integral=0;//�⻷������
float YawOutPID_P=0,YawOutPID_I=0,YawOutPID_D=0;//�⻷PID_P��,�⻷PID_I��,�⻷PID_D��
float YawOutPID_Out=0;//�⻷PID���
float YawOut_Diff=0;//�⻷�Ƕ�΢��

/************************�ڻ�����********************************/
float YawInsdErr_Now=0,YawInsdErr_Last=0;//Pitch�ᵱǰ���ٶ����,Pitch���ϴν��ٶ����
float YawInsdErr_Integral=0;//�ڻ�������
float YawInsdPID_P=0,YawInsdPID_I=0,YawInsdPID_D=0;//�ڻ�PID_P��,�ڻ�PID_I��,�ڻ�PID_D��
float YawInsd_Diff=0;//�ڻ����ٶ�΢��
	
//***************Yaw����PID������***********************//
//Out_Target		�⻷Ŀ��Ƕ�--�û�����
//Out_Measure		�⻷ʵ�ʽǶ�--�����ǽ���ֵ�����������ֵ
//Inside_Measure	�ڻ�ʵ���ٶ�--�����ǽ��ٶ�ֵ
//Inside_Output		Ψһ���ֵ--���͵����
//����ֵ:��
void YawCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output)
{
	/**************�⻷PI*************/
	YawOutErr_Now=Out_Measure-Out_Target;//��ǰ�Ƕ����
	YawOutPID_P=PID_Parameters.Yaw.Kp*YawOutErr_Now;//�⻷PID_P��
	YawOutErr_Integral+=YawOutErr_Now;		//��ǰ�Ƕ�������
	//��ǰ�Ƕ��������޷�
	if(YawOutErr_Integral>28000)		YawOutErr_Integral=28000;
	else if(YawOutErr_Integral<-28000)	YawOutErr_Integral=-28000;
	
	YawOutPID_I=PID_Parameters.Yaw.Ki*YawOutErr_Integral;//�⻷PID_I��
	YawOut_Diff=YawOutErr_Now-YawOutErr_Last;//�Ƕ�΢��
	YawOutErr_Last=YawOutErr_Now;//�����ϴνǶ�ƫ��
	YawInsdPID_D=PID_Parameters.Yaw.Kd*YawOut_Diff;//�⻷PID_D��
	YawOutPID_Out=YawOutPID_P+YawOutPID_I+YawOutPID_D;//�⻷PID���
	
	/**************�ڻ�PID*************/
	YawInsdErr_Now=YawOutPID_Out-Inside_Measure;//��ǰ���ٶ����	
//	YawInsdErr_Now=map(RC_Data.RC.ch0,364,1684,-200,200)-Inside_Measure;//��ǰ���ٶ����	//�����ٶȻ���
	
	YawInsdPID_P=PID_Parameters.Yaw.KKp*YawInsdErr_Now;//�ڻ�PID_P��
	YawInsdErr_Integral+=YawInsdErr_Now;//��ǰ���ٶ�������
	//��ǰ�Ƕ��������޷�
	if(YawInsdErr_Integral>28000)			YawInsdErr_Integral=28000;
	else if(YawInsdErr_Integral<-28000)		YawInsdErr_Integral=-28000;
	
	YawInsdPID_I=PID_Parameters.Yaw.KKi*YawInsdErr_Integral;//�ڻ�PID_I��
	YawInsd_Diff=YawInsdErr_Now-YawInsdErr_Last;//��ǰ���ٶ�΢��
	YawInsdErr_Last=YawInsdErr_Now;//�����ϴν��ٶ����
	YawInsdPID_D=PID_Parameters.Yaw.KKd*YawInsd_Diff;//�ڻ�PID_D��
	
	*Inside_Output=YawInsdPID_P+YawInsdPID_I+YawInsdPID_D;
}


#else

//�⻷����
float Pitch_Kp=10;//10;
float Pitch_Ki=0.02;//0.1;
float Pitch_Kd=16;//12;
//�ڻ�����
float Pitch_KKp=8.0;
float Pitch_KKi=0;
float Pitch_KKd=20;

float Pitch_i;          //������
float Pitch_old;        //�Ƕȱ���
float Pitch_d;          //΢����
float Pitch_shell_out;//�⻷�����

float Gyro_radian_old_y;//�����Ǳ���
float pitch_core_kp_out,pitch_core_kd_out;//�ڻ��������
float Pitch_core_out;//�ڻ������

//***************Pitch����PID������***********************//
//Out_Target		�⻷Ŀ��Ƕ�--�û�����
//Out_Measure		�⻷ʵ�ʽǶ�--�����ǽ���ֵ�����������ֵ
//Inside_Measure	�ڻ�ʵ���ٶ�--�����ǽ��ٶ�ֵ
//Inside_Output		Ψһ���ֵ--���͵����
//����ֵ:��
/*********************************/
void PitchCascade_PID(float Out_Target,float Out_Measure,float Inside_Measure,int16_t* Inside_Output)
{
	////////////////////////�⻷�ǶȻ�(PID)///////////////////////////////
	Pitch_i+=(Out_Measure-Out_Target);
	//-------------Pitch�����޷�----------------//
	if(Pitch_i>3000) Pitch_i=3000;
	else if(Pitch_i<-3000) Pitch_i=-3000;
	//-------------Pitch΢��--------------------//
	Pitch_d=Out_Measure-Pitch_old;
	//-------------Pitch  PID-------------------//
	Pitch_shell_out = Pitch_Kp*(Out_Measure-Out_Target) + Pitch_Ki*Pitch_i + Pitch_Kd*Pitch_d;
	//�Ƕȱ���
	Pitch_old=Out_Measure;
	/*********************************************************/
	////////////////////////�ڻ����ٶȻ�(PD)///////////////////////////////
	pitch_core_kp_out = Pitch_KKp * (Pitch_shell_out + Inside_Measure);
	pitch_core_kd_out = Pitch_KKd * (Inside_Measure  - Gyro_radian_old_y);

	Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
	Gyro_radian_old_y = Inside_Measure;//������ʷֵ
	
	*Inside_Output=Pitch_core_out;
}

#endif

