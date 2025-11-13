#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include "PID_Control.h"

#define  limit            6.3
#define  Output_ramp      10000

//限幅
float  _constrain(float amt, float low, float high)    
{
	return ((amt<low)?(low):((amt)>(high)?(high):(amt)));
}


float PID_Controller(struct _PID *pid, float error) 
{
    float Ts = 0.0;
    uint32_t Timestamp  = SysTick->VAL; // 假设这里是正确获取时间戳的方式
    if (Timestamp < pid->Timestamp_Last) 
        Ts = (float)(pid->Timestamp_Last - Timestamp) / 9 * 1e-6;
    else
        Ts = (0xFFFFFF - Timestamp + pid->Timestamp_Last) / 9 * 1e-6;

    if (Ts <= 0 || Ts > 0.05f) Ts = 0.001;

    float proportion = pid->Kp * error; // P环

    float intergration = pid->Last_intergration + pid->Ki * 0.5f * Ts * error; // I环
    // 假设 _constrain 函数可以对 intergration 进行限制
		intergration = _constrain(intergration, -limit, limit);
    float differential = pid->Kd * (error - pid->Last_Error) / Ts; // D环

    float Output = proportion + intergration + differential;
    // 假设 _constrain 函数可以对 Output 进行限制
		Output = _constrain(Output, -limit, limit);
    pid->Last_Error = error;
    pid->Last_intergration = intergration;
    pid->Last_Output = Output;
    pid->Timestamp_Last = Timestamp;

    return Output;
}
