#ifndef __DFOC_H
#define __DFOC_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pwm.h"
#include "AS5600.h"
#include "Delay.h"
#include "Serial.h"
#include "Lowpass_Filter.h"
#include "PID_Control.h"
#include "Inlinecurrent.h"

struct Motor_{
	int Mot_num;
	float Ua;
	float Ub;
	float Uc;
	float Ubeta;
	float Ualpha;
	float dc_a;
	float dc_b;
	float dc_c;
};

void Motor_en(void);
float constrain(float amt, float low, float high);
void SetPwm(int Mot_num, float Ua, float Ub, float Uc);
float normalizeAngle(float angle);
void SetPhaseVoltage(struct Motor_ *Motor, float Uq, float angle_el);
float cal_Iq_Id(float current_a,float current_b,float angle_el);
void Check_Sensor(void);
void FOC_Init(float power);
float M0_electricAngle(void);
float M1_electricAngle(void);
void M0_Set_Angle(float Target);
void M0_Set_Velocity(float Target);
void M0_Set_CurTorque(float Target);
void M1_Set_Angle(float Target);
void M1_Set_Velocity(float Target);
void M1_Set_CurTorque(float Target);
float velocityopenloop(float target);
void Print_Velocity(int Motor_Velocity);

extern struct AS5600_Sensor Angle_Sensor0;
extern struct AS5600_Sensor Angle_Sensor1;

extern struct Motor_ M0 ;  
extern struct Motor_ M1 ; 

extern struct _PID M0_VEL_PID ;
extern struct _PID M1_VEL_PID ;

extern struct LOWPASS M0_VEL_Filter ;
extern struct LOWPASS M1_VEL_Filter ;

extern struct AS5600_Sensor Angle_Sensor0 ;
extern struct AS5600_Sensor Angle_Sensor1 ;

#endif


