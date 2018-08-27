#include "stm32f4xx.h" 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 

typedef struct
{
	//PID 三个参数 
	double Kp;
	double Ki;
	double Kd;
 
	double err;
	double last_err;
	double out_kp;
	double out_ki;
	double out_kd;
	double limit_Ki;
	double limit_U;
	double U;
	 
}PidTypeDef;

extern PidTypeDef chassismotor_1;
extern PidTypeDef chassismotor_2;
extern PidTypeDef chassismotor_3;
extern PidTypeDef chassismotor_4;

extern PidTypeDef yawmotor_speed_remote;
extern PidTypeDef yawmotor_speed_auto;
extern PidTypeDef yawmotor_remote;
extern PidTypeDef yawmotor_auto;
extern PidTypeDef pitchmotor_speed_remote;
extern PidTypeDef pitchmotor_speed_auto;
extern PidTypeDef pitchmotor_remote;
extern PidTypeDef pitchmotor_auto;

extern PidTypeDef triggermotor;


void PID_InitALL(void);                //初始化位置环，速度环参数
void PID_Init(PidTypeDef * pid);

void PID_SetParam(PidTypeDef * pid,double p, double i, double d, double limit_Ki , double limit_U);

float PID_Calc(PidTypeDef * pid, double rel_val, double set_val);




