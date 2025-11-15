
/**
Deng's FOC 闭环速度控制例程 测试硬件：DengFOC V3P
在串口窗口中输入：A+速度\n，M0电机闭环转动.    B+速度\n，M1电机闭环转动
比如让M0电机以 10rad/s 的速度转动，则输入：A10\n  
在使用自己的电机时，请一定记得修改默认极对数，即 M0_PP中的值，设置为自己的极对数数字
默认PID针对的电机是 2208 ，使用自己的电机需要修改PID参数，才能实现更好效果
**/
#include "stm32f10x.h"                  // Device header
#include "DFOC.h"
#include "Serial.h"
#include "VMC.h"
#include "pid.h"

#define    PI    3.14159265359f

int M0_PP =7, M0_DIR = -1;
int M1_PP =7, M1_DIR = 1 ;

int Motor0 =0;
int Motor1 = 1;

float real_ph1_temp;
float real_ph1;

float real_ph4_temp;
float real_ph4;

float target_x;
float targrt_y;

float last_torque_0;
float last_torque_1;

int main(void)
{
  Serial_Init(115200);
  Motor_en(); //电机使能
	FOC_Init(12.6);//foc初始化
	Systick_CountMode();  //初始化嘀嗒定时器
	Serial_SendString("INIT_OK\n");
	
	LegVMC_Init(&LEG_DATA);//初始化腿部
	
	LegVMC_SetTarget(&LEG_DATA,0,70);//设置腿部位置
	
	pid_init();//pid初始化
	
	LegVMC_Calc(&LEG_DATA);//预计算腿部数据
	
	while(1)
	{
		
		//角度重映射到水平面
		real_ph1_temp=GetAngle_NoTrack(&Angle_Sensor0)/3.14/2*360-45.7;
		if(real_ph1_temp<0)
		{
			real_ph1=(314.3-real_ph1_temp)* 0.0174533;
		}
		else
		{
			real_ph1=real_ph1_temp* 0.0174533;
		}
		
		real_ph4_temp=GetAngle_NoTrack(&Angle_Sensor1)/3.14/2*360-192.4;
		if(real_ph4_temp>167.6)
		{
			real_ph4=real_ph4_temp* 0.0174533;
		}
		else
		{
			real_ph4=real_ph4_temp* 0.0174533;
		}				
			
		//更新腿部信息
		LegVMC_Calc(&LEG_DATA);
				
//		//发送角度数据
//		Serial_SendFloatNumber(real_ph1/ 0.0174533,3,1);
//		Serial_SendFloatNumber(real_ph4/ 0.0174533,3,1);
		
//		//发送力矩数据
//		Serial_SendFloatNumber(-LEG_DATA.torque[0],3,1);
//		Serial_SendFloatNumber(LEG_DATA.torque[1],3,1);
		
//		//发送pid数据
//		Serial_SendFloatNumber(FX_PID_OUT,3,1);
//		Serial_SendFloatNumber(FY_PID_OUT,3,1);
		
//		//输出坐标数据
//		Serial_SendFloatNumber(LEG_DATA.XC,3,1);
//		Serial_SendFloatNumber(LEG_DATA.YC,3,1);

//		//输出速度环数据
//		Serial_SendFloatNumber(M0_SPEED_PID_OUT,3,1);
//		Serial_SendFloatNumber(M1_SPEED_PID_OUT,3,1);
		
		//输出电机力矩
		SetPhaseVoltage(&M0 , constrain((-LEG_DATA.torque[0]),-6.3,+6.3), M0_electricAngle());
		SetPhaseVoltage(&M1 , constrain((LEG_DATA.torque[1]), -6.3,+6.3),M1_electricAngle());
		
		
//	//输出电机力矩
//	SetPhaseVoltage(&M0 , constrain((-M0_SPEED_PID_OUT),-6.3,+6.3), M0_electricAngle());
//	SetPhaseVoltage(&M1 , constrain((M1_SPEED_PID_OUT), -6.3,+6.3), M1_electricAngle());
		
	}

}
