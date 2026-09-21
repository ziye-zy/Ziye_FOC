#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "arm_math.h"
#include "math.h"
#include "AS5600.h"


#define Rm				6.5f
#define POLE_PAIRS		7
#define TPWM			8400    //中心对称模式故周期为4200x2

// 限幅函数
#define Constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))




void MyFOC_Init();
void MyFOC_PWM_Start();
// void MyFOC_SetPWM1C(uint16_t Num);
// void MyFOC_SetPWM2C(uint16_t Num);
// void MyFOC_SetPWM3C(uint16_t Num);

void Set_PhaseVoltage(float Ud, float Uq);
void Get_PhaseVoltage(float* arr);
void Get_ZeroElAngle();
float Get_Angle();

void Get_idiq(int16_t* iabc,float* ialpha_beta, float* idq);

void Get_Electrical_Angle();

void Current_PI_Control(float* idq_ref, float* idq);

// void Velocity_OpenLoop(float taget_v, float Uq, float Ud);

#endif