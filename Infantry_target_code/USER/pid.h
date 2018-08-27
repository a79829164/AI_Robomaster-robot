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

extern PidTypeDef yawmotor_speed;
extern PidTypeDef yawmotor_angle;
extern PidTypeDef yawmotor_position;
extern PidTypeDef pitchmotor_speed;
extern PidTypeDef pitchmotor_angle;
extern PidTypeDef pitchmotor_position;

extern PidTypeDef triggermotor;


void PID_InitALL(void);                //初始化位置环，速度环参数
void PID_Init(PidTypeDef * pid);

void PID_SetParam(PidTypeDef * pid,double p, double i, double d, double limit_Ki , double limit_U);

float PID_Calc(PidTypeDef * pid, double rel_val, double set_val);




