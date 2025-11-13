#include "pid.h" 
#include "VMC.h"

PID_LocTypeDef FX_PID;//转动力矩pid
PID_LocTypeDef FY_PID;//纵向力矩pid

float FX_PID_OUT;//转动力矩pid输出
float FY_PID_OUT;//纵向力矩pid输出

//pid初始化
void pid_init(void)
{
	FX_PID.kp=0;
	FX_PID.ki=0;
	FX_PID.kd=0;
	FX_PID.PID_I_LIMIT_MAX=0;
	FX_PID.PID_OUT_LIMIT_MAX=0;
	
	FY_PID.kp=0;
	FY_PID.ki=0;
	FY_PID.kd=0;
	FY_PID.PID_I_LIMIT_MAX=0;
	FY_PID.PID_OUT_LIMIT_MAX=0;
}


//位置式pid
float PID_location(float setvalue, float actualvalue, PID_LocTypeDef *PID)
{ 
	PID->ek =setvalue-actualvalue;
	PID->location_sum += PID->ek;//计算累计误差值                         
	if((PID->ki!=0)&&(PID->location_sum*PID->ki>PID->PID_I_LIMIT_MAX)) PID->location_sum=PID->PID_I_LIMIT_MAX;
	if((PID->ki!=0)&&(PID->location_sum*PID->ki<-PID->PID_I_LIMIT_MAX)) PID->location_sum=-PID->PID_I_LIMIT_MAX;
	//out计算
    PID->out=PID->kp*PID->ek+(PID->ki*PID->location_sum)+PID->kd*(PID->ek-PID->ek1);
    PID->ek1 = PID->ek;  
    //PID限幅
	if(PID->out<-PID->PID_OUT_LIMIT_MAX)PID->out=-PID->PID_OUT_LIMIT_MAX;
	if(PID->out>PID->PID_OUT_LIMIT_MAX)PID->out=PID->PID_OUT_LIMIT_MAX;
	
	return PID->out;
}

//FXpid计算函数
void FX_pid_location(void)
{
	FX_PID_OUT=PID_location(LEG_DATA.target_x,LEG_DATA.actual_x,&FX_PID);
}

//FYpid计算函数
void FY_pid_location(void)
{
	FY_PID_OUT=PID_location(LEG_DATA.target_y,LEG_DATA.actual_y,&FY_PID);
}




 
    









