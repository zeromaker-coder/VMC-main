#include "vmc.h"
#include "math.h"
#include "pid.h"

#define PI 3.1415926f

LegVMC_Data LEG_DATA;
extern float real_ph0;
extern float real_ph4;


//VMC初始化
void LegVMC_Init(LegVMC_Data *data) 
{
    data->l1 = 60;
    data->l2 = 93;
    data->l3 = 93;
    data->l4 = 60;
    data->l5 = 39;
    data->phi1 = real_ph0;
    data->phi4 = real_ph4;
}

//设置VMCpid计算目标值
void LegVMC_SetTarget(LegVMC_Data *data, float target_x, float target_y) 
{
    data->target_x = target_x;
    data->target_y = target_y;
}

//设置VMCpid计算实际值
void LegVMC_UpdateActual(LegVMC_Data *data, float actual_x, float actual_y) 
{
    data->actual_x = actual_x;
    data->actual_y = actual_y;
}

//VMC计算函数
void LegVMC_Calc(LegVMC_Data *data) 
{
	
	  data->phi1 = real_ph0;
    data->phi4 = real_ph4;
    
	  //pid计算
    FX_pid_location();
		FY_pid_location();
    
	  //给力矩幅值
    data->F0 = FY_PID_OUT;//纵向力矩
    data->Tp = FX_PID_OUT;//随关节转动力矩
    
	  //计算c点坐标以及雅可比矩阵系数
    data->YB = data->l1 * sinf(data->phi1);
    data->XB = data->l1 * cosf(data->phi1);
    data->YD = data->l4 * sinf(data->phi4);
    data->XD = data->l5 + data->l4 * cosf(data->phi4);
    
    data->lBD = sqrtf((data->XD - data->XB)*(data->XD - data->XB) + 
                     (data->YD - data->YB)*(data->YD - data->YB));
    data->A0 = 2 * data->l2 * (data->XD - data->XB);
    data->B0 = 2 * data->l2 * (data->YD - data->YB);
    data->C0 = data->l2*data->l2 + data->lBD*data->lBD - data->l3*data->l3;
    
    float sqrt_arg = data->A0*data->A0 + data->B0*data->B0 - data->C0*data->C0;
    sqrt_arg = (sqrt_arg < 0.0f) ? 0.0f : sqrt_arg;
    data->phi2 = 2 * atan2f((data->B0 + sqrtf(sqrt_arg)), data->A0 + data->C0);
    data->phi3 = atan2f(data->YB - data->YD + data->l2*sinf(data->phi2),
                       data->XB - data->XD + data->l2*cosf(data->phi2));
    
    data->XC = data->l1*cosf(data->phi1) + data->l2*cosf(data->phi2);
    data->YC = data->l1*sinf(data->phi1) + data->l2*sinf(data->phi2);
		
		//更新c点坐标目标值
		LegVMC_SetTarget(data,data->XC,data->YC);
		
    data->L0 = sqrtf((data->XC - data->l5/2.0f)*(data->XC - data->l5/2.0f) + data->YC*data->YC);
    data->phi0 = atan2f(data->YC, data->XC - data->l5/2.0f);
    
    float sin_phi3_phi2 = sinf(data->phi3 - data->phi2);
    sin_phi3_phi2 = (fabsf(sin_phi3_phi2) < 1e-6f) ? 1e-6f : sin_phi3_phi2;
    
		//计算雅可比矩阵系数
    data->j11 = (data->l1 * sinf(data->phi0 - data->phi3) * sinf(data->phi1 - data->phi2)) / sin_phi3_phi2;
    data->j12 = (data->l1 * cosf(data->phi0 - data->phi3) * sinf(data->phi1 - data->phi2)) / (data->L0 * sin_phi3_phi2);
    data->j21 = (data->l4 * sinf(data->phi0 - data->phi2) * sinf(data->phi3 - data->phi4)) / sin_phi3_phi2;
    data->j22 = (data->l4 * cosf(data->phi0 - data->phi2) * sinf(data->phi3 - data->phi4)) / (data->L0 * sin_phi3_phi2);
    
		//计算输出力矩
    data->torque[0] = data->j11 * data->F0 + data->j12 * data->Tp;
    data->torque[1] = data->j21 * data->F0 + data->j22 * data->Tp;
}