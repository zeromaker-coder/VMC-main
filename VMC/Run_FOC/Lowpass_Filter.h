#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H

struct LOWPASS{
	float Tf;
	float Last_Timesamp;
	float Last_y;
};

float Lowpassfilter_sim(float x);
float Lowpassfilter(struct LOWPASS *lowpass, float x);



#endif


