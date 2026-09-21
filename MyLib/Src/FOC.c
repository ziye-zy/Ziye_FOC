#include "FOC.h"

// -- 电机参数 --

float Rs = 1.55f;
float Ls = 0.0012f;
float Ts = 0.00005f;    //半个采样周期，采样周期为0.00005s
float psi_f = 0.004762f;

// -- 电流环PI --

float Kp_I = 0.0f;
float Ki_I = 0.0f;

float I_Error[2] = {0.0f, 0.0f};
float I_Error_sum[2] = {0.0f, 0.0f};

float Udq_ref[2] = {0.0f, 0.0f};

float Ki_MAX = 5.0f;

// -- --

static float Power_Valtage = 12;

// static float Uq = 0, Ud = 0;
static float Ua = 0, Ub = 0, Uc = 0;
static float Ualpha = 0, Ubeta = 0;

static float Zero_elAngle = 0;
static float angle = 0;     //机械角度 0-2PI
float elAngle = 0;

// float ialpha_beta[2] = {0, 0};

static uint16_t PWM1 = 0;
static uint16_t PWM2 = 0;
static uint16_t PWM3 = 0;

static int8_t Dir = -1;

float sin_elangle = 0, cos_elangle = 0;     // 电角度正余弦值

float sqrt3 = 0;

void MyFOC_PWM_Start()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void MyFOC_Init()
{
	HAL_GPIO_WritePin(PWMEN_GPIO_Port, PWMEN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MRESET_GPIO_Port, MRESET_Pin, GPIO_PIN_SET);
    MyFOC_PWM_Start();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 4200-2);

    sqrt3 = sqrt(3);

    // -- 计算电流环PI --
    // Kp_I = Ls * 0.5f / Ts;
    // Ki_I = Rs * 0.5f / Ts;

    Kp_I = Ls * 0.5f;
    Ki_I = Rs * 0.5f;

    // Kp_I = 1.0f;
    // Ki_I = 1.0f;


}


// void MyFOC_SetPWM1C(uint16_t Num)
// {
// 	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Num);
// }

// void MyFOC_SetPWM2C(uint16_t Num)
// {
// 	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Num);
// }

// void MyFOC_SetPWM3C(uint16_t Num)
// {
// 	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Num);
// }

//角度归一化
float Normalize_Angle(float angle)
{
	float temp = fmod(angle, 2*PI);
	
	return (temp >= 0 ? temp : (temp + 2*PI));
}



//电角度求解
void Get_Electrical_Angle()
{
	angle = AS5600_GetAngleData() / 4096.0f * 2*PI;
		
	// return Dir * angle * POLE_PAIRS - Zero_elAngle;

    elAngle = Normalize_Angle(Dir * angle * POLE_PAIRS - Zero_elAngle);
	
	sin_elangle = arm_sin_f32(elAngle);
	cos_elangle = arm_cos_f32(elAngle);
}

float Get_Angle()
{
	return angle;
}

void Set_PWM(float Ua, float Ub, float Uc)
{
	// MyFOC_PWM_Start();
	
	PWM1 = TPWM * Constrain(Ua/Power_Valtage,0.0f,1.0f);
	PWM2 = TPWM * Constrain(Ub/Power_Valtage,0.0f,1.0f);
	PWM3 = TPWM * Constrain(Uc/Power_Valtage,0.0f,1.0f);
	
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM3);

	// MyFOC_SetPWM1C(PWM1);
	// MyFOC_SetPWM2C(PWM2);
	// MyFOC_SetPWM3C(PWM3);
}

//设置相电压(SPWM)
void Set_PhaseVoltage(float Ud, float Uq)
{	
	
	Ualpha = Ud * cos_elangle - Uq * sin_elangle;
	Ubeta = Uq *cos_elangle + Ud * sin_elangle;
	
	

	//克拉克逆变换
	Ua = Ualpha + Power_Valtage*0.5f;
	Ub = (sqrt3 * Ubeta - Ualpha) * 0.5f +  Power_Valtage* 0.5f;
	Uc = -(Ualpha + sqrt3 * Ubeta) * 0.5f +  Power_Valtage* 0.5f;
	
	
	
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
    Delay_ms(500);
    elAngle = 3*PI/2;
    sin_elangle = arm_sin_f32(elAngle);
	cos_elangle = arm_cos_f32(elAngle);
    Delay_ms(500);
	Set_PhaseVoltage(0, 3);
	Delay_ms(1000);
    Get_Electrical_Angle();
    Delay_ms(500);
	Zero_elAngle = elAngle;
	Set_PhaseVoltage(0, 0);
}

// void Velocity_OpenLoop(float taget_v, float Uq, float Ud)
// {
// 	static uint32_t Last_ms;
// 	static float Motor_Angle;
// 	uint32_t Now_ms = HAL_GetTick();
	
// 	float Ts = (Now_ms - Last_ms) * 1e-3f;
	
// 	Last_ms = Now_ms;
	
// 	if(Ts <= 0) Ts = 1e-3f;
	
// 	Motor_Angle = Normalize_Angle(Motor_Angle + Ts * taget_v);
		
// 	Set_PhaseVoltage(Uq, Ud, Motor_Angle*POLE_PAIRS);
// }

//clark与park变换获取idiq
void Get_idiq(int16_t* iabc, float* ialpha_beta, float* idq)
{
    ialpha_beta[0] = (2 * iabc[0] - iabc[1] - iabc[2])/3.0f;
    ialpha_beta[1] = (iabc[1] - iabc[2])*3.0f/sqrt3;

    idq[0] = (ialpha_beta[0]*cos_elangle + ialpha_beta[1]*sin_elangle)/2048.0f;
    idq[1] = (-ialpha_beta[0]*sin_elangle + ialpha_beta[1]*cos_elangle)/2048.0f;
}

// 电流环PI控制

void Current_PI_Control(float* idq_ref, float* idq)
{
    I_Error[0] = idq_ref[0] - idq[0];
    I_Error[1] = idq_ref[1] - idq[1];

    I_Error_sum[0] += I_Error[0];
    I_Error_sum[1] += I_Error[1];

    Udq_ref[0] = Kp_I * I_Error[0] + Constrain(Ki_I * I_Error_sum[0],-Ki_MAX,Ki_MAX);
    Udq_ref[1] = Kp_I * I_Error[1] + Constrain(Ki_I * I_Error_sum[1],-Ki_MAX,Ki_MAX);

    Set_PhaseVoltage(Constrain(Udq_ref[0],-Ki_MAX,Ki_MAX), Constrain(Udq_ref[1],-Ki_MAX,Ki_MAX));
}