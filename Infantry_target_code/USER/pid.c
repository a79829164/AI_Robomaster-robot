#include "pid.h"

PidTypeDef chassismotor_1;
PidTypeDef chassismotor_2;
PidTypeDef chassismotor_3;
PidTypeDef chassismotor_4;

PidTypeDef yawmotor_speed;
PidTypeDef yawmotor_angle;
PidTypeDef yawmotor_position;
PidTypeDef pitchmotor_speed;
PidTypeDef pitchmotor_angle;
PidTypeDef pitchmotor_position;

PidTypeDef triggermotor;



//初始化PID 
void PID_Init(PidTypeDef * pid)
{
	memset(pid, 0, sizeof(PidTypeDef));
}


void PID_InitALL(void)                //初始化位置环，速度环参数
{
	PID_Init(&chassismotor_1);
 	PID_SetParam(&chassismotor_1 , 10 , 0.001 , 10 , 500 , 10000); 
	PID_Init(&chassismotor_2);
 	PID_SetParam(&chassismotor_2 , 10 , 0.001 , 10 , 500 , 10000);		            
	PID_Init(&chassismotor_3);
 	PID_SetParam(&chassismotor_3 , 10 , 0.001 , 10 , 500 , 10000);		             
	PID_Init(&chassismotor_4);
 	PID_SetParam(&chassismotor_4 , 10 , 0.001 , 10 , 500 , 10000);		
	
	PID_Init(&yawmotor_speed);
 	PID_SetParam(&yawmotor_speed , 1000 , 0.001 , 150 , 100 , 10000);	
	PID_Init(&yawmotor_angle);
 	PID_SetParam(&yawmotor_angle , 0.5 , 0.001 , 0 , 0.8 , 2.5);	
  PID_Init(&yawmotor_position);
 	PID_SetParam(&yawmotor_position , 0.003 , 0.001 , 0 , 0.12 , 1.2);	
	
	PID_Init(&pitchmotor_speed);
 	PID_SetParam(&pitchmotor_speed , 1000 , 0.001 , 150 , 100 , 10000);	
	PID_Init(&pitchmotor_angle);
 	PID_SetParam(&pitchmotor_angle , 0.02 , 0.001 , 0 , 1 , 3);	
	PID_Init(&pitchmotor_position);
 	PID_SetParam(&pitchmotor_position , 0.003 , 0.001 , 0 , 0.56 , 1.6);	
	
	PID_Init(&triggermotor);
 	PID_SetParam(&triggermotor , 1 , 0.01 , 10 , 1500 , 10000);		
}

//设置参数 
void PID_SetParam(PidTypeDef * pid,double p, double i, double d, double limit_Ki , double limit_U)
{
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;
  pid->limit_Ki = limit_Ki;
	pid->limit_U = limit_U;
}

//PID计算 
float PID_Calc(PidTypeDef * pid, double rel_val, double set_val)
{
	double p = 0,
         i = 0,
         d = 0;
	///////////
	pid->err = set_val - rel_val;
	
	///////////
	pid->out_kp = pid->err * pid->Kp;
	
	///////////
	pid->out_ki += pid->err * pid->Ki;
	if(pid->out_ki >= pid->limit_Ki)
	{
		pid->out_ki = pid->limit_Ki;
	}
  else if(pid->out_ki <= -pid->limit_Ki)
	{
		pid->out_ki = -pid->limit_Ki;
	}
	
	///////////
	pid->out_kd =  -(pid->last_err - pid->err) * pid->Kd;
	pid->last_err = pid->err;
	
	///////////
	pid->U = pid->out_kp + pid->out_ki + pid->out_kd;
	if(pid->U >= pid->limit_U)
	{
		pid->U = pid->limit_U;
	}
  else if(pid->U <= -pid->limit_U)
	{
		pid->U = -pid->limit_U;
	}
	
	return pid->U;
}
















