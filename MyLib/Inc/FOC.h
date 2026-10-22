#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "arm_math.h"
#include "math.h"
#include "AS5600.h"

// --- 电机参数 ---

#define Rs				1.55f
#define POLE_PAIRS		7
#define TPWM			8400.0f        //中心对称模式故周期为4200x2
#define Ls              0.0012f
#define Ts              0.000025f   //半个采样周期，采样周期为0.00005s
#define psi_f           0.004762f


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
uint16_t Get_Angle_raw();

void Get_CCR(uint16_t* ccr);

void Get_idiq(int16_t* iabc,float* ialpha_beta, float* idq);

void Get_Electrical_Angle();
void Pre_Electrical_Angle(uint16_t motor_raw_angle, uint16_t time, float motor_speed);

void Current_PI_Control(float* idq_ref, float* idq);

float Speed_PI_Control(float speed_ref, float speed);

// void Velocity_OpenLoop(float taget_v, float Uq, float Ud);

#endif