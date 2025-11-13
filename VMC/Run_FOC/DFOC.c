#include "stm32f10x.h"
#include "DFOC.h"

#define PI         3.14159265359f
#define _3PI_2     4.71238898f
#define _1_SQRT3 	 0.57735026919f
#define _2_SQRT3   1.15470053838f

struct Motor_ M0 = {0};  
struct Motor_ M1 = {1}; 

struct _PID M0_VEL_PID = {0.4, 2, 0};
struct _PID M1_VEL_PID = {0.4, 2, 0};

struct LOWPASS M0_VEL_Filter = {0.1};
struct LOWPASS M1_VEL_Filter = {0.1};

struct AS5600_Sensor Angle_Sensor0 = {0};
struct AS5600_Sensor Angle_Sensor1 = {1};
  
float voltage_limit = 12;
float voltage_power_supply = 0;
float M0_zero_elc_Angle = 0, M1_zero_elc_Angle = 0;
extern int M0_PP , M0_DIR ;
extern int M1_PP , M1_DIR ;



void Motor_en()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

//限制幅值
float constrain(float amt, float low, float high)
{
	 return ((amt<low)?(low):((amt)>(high)?(high):(amt)));
}

//将角度归化到0-2PI
float normalizeAngle(float angle)
{
	float a = fmod(angle, 2*PI);
	return ((a>=0) ? a : (a + 2*PI));
}

float M0_electricAngle(void)
{
	return normalizeAngle((GetAngle_NoTrack(&Angle_Sensor0) * M0_PP * M0_DIR) - M0_zero_elc_Angle);
}

float M1_electricAngle(void)
{
	return normalizeAngle((GetAngle_NoTrack(&Angle_Sensor1) * M1_PP * M1_DIR) - M1_zero_elc_Angle);
}

void SetPwm(int Mot_num, float Ua, float Ub, float Uc)
{
	float U_a=0.0;
	float U_b=0.0;
	float U_c=0.0;
	
	U_a = constrain(Ua, 0.0f, voltage_limit);
	U_b = constrain(Ub, 0.0f, voltage_limit);
	U_c = constrain(Uc, 0.0f, voltage_limit);
	
	if(Mot_num == 0)
	{
		M0.dc_a = constrain(U_a / voltage_power_supply, 0.0f, 1.0f);
		M0.dc_b = constrain(U_b / voltage_power_supply, 0.0f, 1.0f);
		M0.dc_c = constrain(U_c / voltage_power_supply, 0.0f, 1.0f);
		
		M0_PWM_A(M0.dc_a * 4800.0f);  // 频率15k
		M0_PWM_B(M0.dc_b * 4800.0f);
		M0_PWM_C(M0.dc_c * 4800.0f);
	}
	else if(Mot_num == 1)
	{
		M1.dc_a = constrain(U_a / voltage_power_supply, 0.0f, 1.0f);
		M1.dc_b = constrain(U_b / voltage_power_supply, 0.0f, 1.0f);
		M1.dc_c = constrain(U_c / voltage_power_supply, 0.0f, 1.0f);
		
		M1_PWM_A(M1.dc_a * 4800.0f);  // 频率15k
		M1_PWM_B(M1.dc_b * 4800.0f);
		M1_PWM_C(M1.dc_c * 4800.0f);
	}
}

//FOC核心算法，克拉克逆变换/帕克逆变换
void SetPhaseVoltage(struct Motor_ *Motor, float Uq, float angle_el)
{
//	angle_el = normalizeAngle(angle_el);
	
	Motor->Ualpha = -Uq*sin(angle_el);
	Motor->Ubeta = Uq*cos(angle_el);
	
	Motor->Ua = Motor->Ualpha + voltage_power_supply / 2;
	Motor->Ub = (sqrt(3)*Motor->Ubeta - Motor->Ualpha) / 2 + voltage_power_supply / 2;
	Motor->Uc = -(Motor->Ualpha + sqrt(3)*Motor->Ubeta) / 2 + voltage_power_supply / 2;
	
	SetPwm(Motor->Mot_num, Motor->Ua,Motor->Ub,Motor->Uc);
}

void Check_Sensor(void)
{
	SetPhaseVoltage(&M0, 3, _3PI_2);
	delay_ms(2000);
	M0_zero_elc_Angle = M0_electricAngle();
	SetPhaseVoltage(&M0, 0, _3PI_2);
	delay_ms(500);
	
	SetPhaseVoltage(&M1, 3, _3PI_2);
	delay_ms(2000);
	M1_zero_elc_Angle = M1_electricAngle();
	SetPhaseVoltage(&M1, 0, _3PI_2);
	delay_ms(500);
}

void FOC_Init(float power)
{
	voltage_power_supply = power;
	PWM_Init();

	AS5600_Init();
	
	Check_Sensor();
}

void Print_Velocity(int Motor_Velocity) 
{
	if(Motor_Velocity == 0)
	{
		 Serial_SendFloatNumber(GetVelocity(&Angle_Sensor0),3, 2);
	}
	if(Motor_Velocity == 1)
	{
		 Serial_SendFloatNumber(GetVelocity(&Angle_Sensor1),3, 2);
	}
}
// 单速度环
void M0_Set_Velocity(float Target)
{
	Angle_Sensor0.velocity = Lowpassfilter(&M0_VEL_Filter, GetVelocity(&Angle_Sensor0));
	SetPhaseVoltage(&M0, PID_Controller(&M0_VEL_PID, M0_DIR*(Target - Angle_Sensor0.velocity)), M0_electricAngle());

}

void M1_Set_Velocity(float Target)
{
	Angle_Sensor1.velocity = Lowpassfilter(&M1_VEL_Filter, GetVelocity(&Angle_Sensor1));
	SetPhaseVoltage(&M1, PID_Controller(&M1_VEL_PID, M1_DIR*(Target - Angle_Sensor1.velocity)), M1_electricAngle());
}







