#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

struct _PID{
	float Kp;
	float Ki;
	float Kd;
	unsigned long Timestamp_Last;
	float Last_Error;
	float Last_intergration;
	float Last_Output;
};

float PID_Controller(struct _PID *pid, float error);
float  _constrain(float amt, float low, float high);

#endif


