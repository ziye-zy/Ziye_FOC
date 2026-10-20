#include "FOC.h"
#include <stdint.h>

// -- 电流环PI --

float Kp_I = 0.0f;
float Ki_I = 0.0f;

float I_Error[2] = {0.0f, 0.0f};
float I_Error_sum[2] = {0.0f, 0.0f};

float Udq_ref[2] = {0.0f, 0.0f};

float Ki_MAX = 5.0f;
float Udq_MAX = 5.0f;


// -- SVPWM --

static uint8_t N = 0;
static float UABC[3] = {0.0f, 0.0f, 0.0f};

static float XYZ[3] = {0.0f, 0.0f, 0.0f};
static float SVPWM_a = 0.0f;  //矢量作用时间系数
static float T[2] = {0.0f, 0.0f}; //基本矢量作用时间
static uint16_t CCR[3] = {0, 0, 0};
static float Tabc[3] = {0.0f, 0.0f, 0.0f};

static float Udc = 12.0f;

// static float Uq = 0, Ud = 0;
static float Ua = 0, Ub = 0, Uc = 0;
static float Ualpha = 0, Ubeta = 0;

static float Zero_elAngle = 0.0f;
static float angle = 0.0f;     //处理后的机械角度 0-2PI
static uint16_t angle_raw = 0;  // 角度原始值
static float elAngle = 0.0f;

// float ialpha_beta[2] = {0, 0};

static uint16_t PWM1 = 0;
static uint16_t PWM2 = 0;
static uint16_t PWM3 = 0;

static int8_t Dir = -1;

static float sin_elangle = 0, cos_elangle = 0;     // 电角度正余弦值

static float sqrt3 = 0;

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

    sqrt3 = sqrt(3.0f);

    // -- 计算电流环PI --
    // Kp_I = Ls * 0.5f / Ts;
    // Ki_I = Rs * 0.5f / Ts;

    Kp_I = Ls * 0.25f;
    Ki_I = Rs * 0.25f;

    // Kp_I = 1.0f;
    // Ki_I = 1.0f;

    // --- 计算SVPWM矢量作用时间系数
    SVPWM_a = sqrt3 * TPWM / Udc;

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

uint16_t recordbuf[256];
float recordbuf1[256];
// void record()
// {
//     pointer++;

//     recordbuf[pointer&0xFF] = angle;
//     recodebuf1[pointer&0xFF] = elAngle;
// }

//电角度求解
void Get_Electrical_Angle()
{
	angle_raw = AS5600_GetAngleData();

    angle = angle_raw / 4096.0f * 2*PI;
		
	// return Dir * angle * POLE_PAIRS - Zero_elAngle;

    elAngle = Normalize_Angle(Dir * angle * POLE_PAIRS - Zero_elAngle);
	
    // record();//出现突变停止

	sin_elangle = arm_sin_f32(elAngle);
	cos_elangle = arm_cos_f32(elAngle);
}



float Get_Angle()
{
	return angle;
}

uint16_t Get_Angle_raw()
{
	return angle_raw;
}

void Set_PWM(float Ua, float Ub, float Uc)
{
	// MyFOC_PWM_Start();
	
	PWM1 = TPWM * Constrain(Ua/Udc,0.0f,1.0f);
	PWM2 = TPWM * Constrain(Ub/Udc,0.0f,1.0f);
	PWM3 = TPWM * Constrain(Uc/Udc,0.0f,1.0f);
	
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
	
	

	// --- 克拉克逆变换(SPWM) ---

	// Ua = Ualpha + Udc*0.5f;
	// Ub = (sqrt3 * Ubeta - Ualpha) * 0.5f +  Udc* 0.5f;
	// Uc = -(Ualpha + sqrt3 * Ubeta) * 0.5f +  Udc* 0.5f;
    // Set_PWM(Ua, Ub, Uc);


	// --- SVPWM ---

    UABC[0] = Ubeta;
    UABC[1] = (sqrt3 * Ualpha - Ubeta)*0.5f;
    UABC[2] = (-sqrt3 * Ualpha - Ubeta)*0.5f;

    // 扇区判断
    N = 0;
    if(UABC[0] > 0) N += 1;
    if(UABC[1] > 0) N += 2;
    if(UABC[2] > 0) N += 4;

    //作用时间计算
	XYZ[0] = SVPWM_a * UABC[0];
    XYZ[1] = -SVPWM_a * UABC[2];
    XYZ[2] = -SVPWM_a * UABC[1];

    switch (N) 
    {
        case 3:  // 扇区 I
            T[0] = -XYZ[2];
            T[1] = XYZ[0]; 
            break;
        case 1:  // 扇区 II
            T[0] = XYZ[2];
            T[1] = XYZ[1]; 
            break;
        case 5:  // 扇区 III
            T[0] = XYZ[0];
            T[1] = -XYZ[1]; 
            break;
        case 4:  // 扇区 IV
            T[0] = -XYZ[0];
            T[1] = XYZ[2]; 
            break;
        case 6:  // 扇区 V
            T[0] = -XYZ[1];
            T[1] = -XYZ[2]; 
            break;
        case 2:  // 扇区 VI
            T[0] = XYZ[1];
            T[1] = -XYZ[0]; 
            break;
        default:
            T[0] = 0;
            T[1] = 0;
            break;
    }
    
    //CCR计算
    Tabc[0] = Constrain((TPWM - T[0] - T[1])*0.25f + 0.5f, 0.0f, TPWM*0.5f);
    Tabc[1] = Constrain(Tabc[0] + T[0]*0.5f + 0.5f, 0.0f, TPWM*0.5f);
    Tabc[2] = Constrain(Tabc[1] + T[1]*0.5f + 0.5f, 0.0f, TPWM*0.5f);

    switch (N) 
    {
        case 3:  // 扇区 I
            CCR[0] = Tabc[0];
            CCR[1] = Tabc[1];
            CCR[2] = Tabc[2];
            break;
        case 1:  // 扇区 II
            CCR[0] = Tabc[1];
            CCR[1] = Tabc[0];
            CCR[2] = Tabc[2];
            break;
        case 5:  // 扇区 III
            CCR[0] = Tabc[2];
            CCR[1] = Tabc[0];
            CCR[2] = Tabc[1];
            break;
        case 4:  // 扇区 IV
            CCR[0] = Tabc[2];
            CCR[1] = Tabc[1];
            CCR[2] = Tabc[0];
            break;
        case 6:  // 扇区 V
            CCR[0] = Tabc[1];
            CCR[1] = Tabc[2];
            CCR[2] = Tabc[0];
            break;
        case 2:  // 扇区 VI
            CCR[0] = Tabc[0];
            CCR[1] = Tabc[2];
            CCR[2] = Tabc[1];
            break;
        default:
            CCR[0] = 0;
            CCR[1] = 0;
            CCR[2] = 0;
            break;
    }


    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CCR[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CCR[1]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, CCR[2]);

}


void Get_CCR(uint16_t* ccr)
{
	for (int i = 0; i < 3; i++)
	{
		ccr[i] = CCR[i];
	}
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

    Set_PhaseVoltage(Constrain(Udq_ref[0],-Udq_MAX,Udq_MAX), Constrain(Udq_ref[1],-Udq_MAX,Udq_MAX));
}