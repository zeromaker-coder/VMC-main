#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include "math.h"
#include "Lowpass_Filter.h"

#define _2PI       6.28318530718f

float y = 0;
float Lowpassfilter_sim(float x)
{
	float out = 0.9*x + 0.1*y;
	y = x;
	return out;
}

float Lowpassfilter(struct LOWPASS *lowpass, float x)
{
	float dt = 0.0;

	uint32_t Timesamp = SysTick->VAL;
	if(Timesamp < lowpass->Last_Timesamp) dt = (float)(lowpass->Last_Timesamp - Timesamp)/9*1e-6;
	else
		dt = (float)(0xFFFFFF - Timesamp + lowpass->Last_Timesamp)/9*1e-6;

	if(dt<0.0||dt==0) dt = 0.0015f;
	else if(dt>0.005f)
	{
		lowpass->Last_y = x;
		lowpass->Last_Timesamp = Timesamp;
		return x;
	}
	float alpha = lowpass->Tf / (lowpass->Tf + dt);
	float y = alpha * lowpass->Last_y + (1.0f - alpha) * x;
	
	lowpass->Last_y = y;
	lowpass->Last_Timesamp = Timesamp;

	return y;
}


