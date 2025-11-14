#ifndef __PID_H
#define __PID_H

typedef struct
{
  float kp;                     
  float ki;                       
  float kd;                       
  float ek;                      
  float ek1;                                            
  float location_sum;            
  float out;
  float PID_I_LIMIT_MAX;
  float PID_OUT_LIMIT_MAX;											
}PID_LocTypeDef;

void FX_pid_location(void);

void FY_pid_location(void);

void pid_init(void);

void M0_speed_pid_location(void);

void M1_speed_pid_location(void);

extern float FX_PID_OUT;

extern float FY_PID_OUT;

extern float M0_SPEED_PID_OUT;
\
extern float M1_SPEED_PID_OUT;


#endif