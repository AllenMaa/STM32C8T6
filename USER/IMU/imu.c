#include "imu.h"
#include <math.h>
#include "dev_mpu6050.h"
#include "usart.h"
//#include "timer.h"


//#include "rc.h"
//#include "pid.h"

AHRS_DATA	AHRS_Data;

//-------------------------------һ����Ԫ��------------------------------------
// ��������

#define Kp 100.0f                // ��������֧�������������ٶȼ�/��ǿ��
#define Ki 0.002f                // ��������֧���ʵ�������ƫ�����ν�
#define halfT 0.00006f             // �������ڵ�һ��

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // ��Ԫ����Ԫ�أ�������Ʒ���
float exInt = 0, eyInt = 0, ezInt = 0;        // ��������С�������

float last_yaw_temp=0,yaw_temp=0;
static int yaw_count=0; 

void IMUupdate(float ax, float ay, float az,float gx, float gy, float gz)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  

        // ����������
        norm = sqrt(ax*ax + ay*ay + az*az);      
        ax = ax / norm;                   //��λ��
        ay = ay / norm;
        az = az / norm;      

        // ���Ʒ��������
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // ���������ͷ��򴫸��������ο�����֮��Ľ���˻����ܺ�
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);

        // ������������������
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

        // ������������ǲ���
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

        // ������Ԫ���ʺ�������
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

        // ��������Ԫ
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;

        AHRS_Data.Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // Pitch ,ת��Ϊ����
        AHRS_Data.Roll	= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // Roll ,ת��Ϊ����
		//yaw��ǶȾ����������������
		last_yaw_temp=yaw_temp;
		yaw_temp = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3; // Yaw ,ת��Ϊ����
		if(yaw_temp-last_yaw_temp>=330)  
		{
			yaw_count--;
		}
		else if (yaw_temp-last_yaw_temp<=-330)
		{
			yaw_count++;
		}
		AHRS_Data.Yaw=yaw_temp + yaw_count*360;  //yaw��Ƕ�
}

#if Report
//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{
	while((USART1->SR&0X40)==0){}	//ѭ������,ֱ���������   
    USART1->DR=c;  
} 
//�������ݸ������������վ(V4�汾)
//fun:������. 0X01~0X1C
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0XAA;	//֡ͷ
	send_buf[1]=0XAA;	//֡ͷ
	send_buf[2]=fun;	//������
	send_buf[3]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//��������
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//����У���	
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}

//���ͼ��ٶȴ���������+����������(������֡)
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ 
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[18]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	tbuf[12]=0;//��Ϊ����MPL��,�޷�ֱ�Ӷ�ȡ����������,��������ֱ�����ε�.��0���.
	tbuf[13]=0;
	tbuf[14]=0;
	tbuf[15]=0;
	tbuf[16]=0;
	tbuf[17]=0;
	usart1_niming_report(0X02,tbuf,18);//������֡,0X02
}	

//ͨ������1�ϱ���������̬���ݸ�����(״̬֡)
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
//csb:�������߶�,��λ:cm
//prs:��ѹ�Ƹ߶�,��λ:mm
void usart1_report_imu(short roll,short pitch,short yaw,short csb,int prs)
{
	u8 tbuf[12];   	
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	tbuf[6]=(csb>>8)&0XFF;
	tbuf[7]=csb&0XFF;
	tbuf[8]=(prs>>24)&0XFF;
	tbuf[9]=(prs>>16)&0XFF;
	tbuf[10]=(prs>>8)&0XFF;
	tbuf[11]=prs&0XFF;
	usart1_niming_report(0X01,tbuf,12);//״̬֡,0X01
}   
#endif




void IMU_Init(void)
{
	MPU6050_Init();
	printf("\r IMU ��ʼ������ \r\n");
	IMU_Sta=1;
}

static float map(long x, long in_min, long in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}    

//��ȡIMU����
void Get_IMU_Data(void)
{
	//��ȡ�˲�������������
	MPU6050_getMotion6(&MPU6050_Data.ax,&MPU6050_Data.ay,&MPU6050_Data.az,&MPU6050_Data.gx,&MPU6050_Data.gy,&MPU6050_Data.gz);
	//��̬����
	IMUupdate(MPU6050_Data.ax,MPU6050_Data.ay,MPU6050_Data.az,MPU6050_Data.gx,MPU6050_Data.gy,MPU6050_Data.gz);
	
#if Report
	//������������λ��V4
	usart1_report_imu((int)(AHRS_Data.Roll*100),(int)(-AHRS_Data.Pitch*100),(int)(AHRS_Data.Yaw*100),0,0);	
	//mpu6050_send_data(Gimbal_Data.Gimbal_Out.Pitch,map(RC_Data.RC.ch0,364,1684,-200,200),MPU6050_Data.gz,0,0,0);
#endif	
}
	
	


