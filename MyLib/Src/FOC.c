#include "FOC.h"

static float Power_Valtage = 12;

// static float Uq = 0, Ud = 0;
static float Ua = 0, Ub = 0, Uc = 0;
static float Ualpha = 0, Ubeta = 0;

static float Zero_elAngle = 0;
static float angle;


static uint16_t PWM1 = 0;
static uint16_t PWM2 = 0;
static uint16_t PWM3 = 0;

static int8_t Dir = -1;

void MyFOC_Init()
{
	HAL_GPIO_WritePin(PWMEN_GPIO_Port, PWMEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MRESET_GPIO_Port, MRESET_Pin, GPIO_PIN_SET);
}

void MyFOC_PWM_Start()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void MyFOC_SetPWM1C(uint16_t Num)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Num);
}

void MyFOC_SetPWM2C(uint16_t Num)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Num);
}

void MyFOC_SetPWM3C(uint16_t Num)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Num);
}

//角度归一化
float Normalize_Angle(float angle)
{
	float temp = fmod(angle, 2*PI);
	
	return (temp >= 0 ? temp : (temp + 2*PI));
}

//电角度求解
float Get_Electrical_Angle()
{
	angle = AS5600_GetAngleData() / 4096.0 * 2*PI;
		
	return Dir * angle * POLE_PAIRS - Zero_elAngle;
}

float Get_Angle()
{
	return angle;
}

void Set_PWM(float Ua, float Ub, float Uc)
{
	MyFOC_PWM_Start();
	
	PWM1 = TPWM * Constrain(Ua/Power_Valtage,0.0f,1.0f);
	PWM2 = TPWM * Constrain(Ub/Power_Valtage,0.0f,1.0f);
	PWM3 = TPWM * Constrain(Uc/Power_Valtage,0.0f,1.0f);
	
	MyFOC_SetPWM1C(PWM1);
	MyFOC_SetPWM2C(PWM2);
	MyFOC_SetPWM3C(PWM3);
}

//设置相电压(正弦注入SVPWM)
void Set_PhaseVoltage(float Uq, float Ud, float elAngle)
{	
//	if(elAngle == -1)
//	{
//		return;
//	}
	
	elAngle = Normalize_Angle(elAngle);
	
	float sin_elangle = 0, cos_elangle = 0;
	float input_value = 0;
	
	//帕克逆变换
//	arm_sin_cos_f32(elAngle, &sin_elangle, &cos_elangle);
	
	sin_elangle = arm_sin_f32(elAngle);
	cos_elangle = arm_cos_f32(elAngle);
	
	Ualpha = Ud * cos_elangle - Uq * sin_elangle;
	Ubeta = Uq *cos_elangle + Ud * sin_elangle;
	
	input_value = -sqrt(Uq*Uq + Ud*Ud)/6*arm_sin_f32(elAngle*3);
	
//	input_value = 0;
	
	//克拉克逆变换
	Ua = Ualpha + input_value + Power_Valtage/2;
	Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + input_value + Power_Valtage/2;
	Uc = -(Ualpha + sqrt(3) * Ubeta) / 2 + input_value + Power_Valtage/2;
	
	
	
	Set_PWM(Ua, Ub, Uc);
}

void Get_PhaseVoltage(float* arr)
{
	arr[0] = Ua;
	arr[1] = Ub;
	arr[2] = Uc;
}

void Get_ZeroElAngle()
{
	Set_PhaseVoltage(3, 0, 3*PI/2);
	Delay_ms(3000);
	Zero_elAngle = Get_Electrical_Angle();
	Set_PhaseVoltage(0, 0, 3*PI/2);
}

void Velocity_OpenLoop(float taget_v, float Uq, float Ud)
{
	static uint32_t Last_ms;
	static float Motor_Angle;
	uint32_t Now_ms = HAL_GetTick();
	
	float Ts = (Now_ms - Last_ms) * 1e-3f;
	
	Last_ms = Now_ms;
	
	if(Ts <= 0) Ts = 1e-3f;
	
	Motor_Angle = Normalize_Angle(Motor_Angle + Ts * taget_v);
		
	Set_PhaseVoltage(Uq, Ud, Motor_Angle*POLE_PAIRS);
}

