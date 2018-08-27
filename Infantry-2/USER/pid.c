#include "pid.h"

PidTypeDef chassismotor_1;
PidTypeDef chassismotor_2;
PidTypeDef chassismotor_3;
PidTypeDef chassismotor_4;

PidTypeDef yawmotor_speed_remote;
PidTypeDef pitchmotor_speed_remote;
PidTypeDef yawmotor_remote;
PidTypeDef pitchmotor_remote;

PidTypeDef yawmotor_auto;
PidTypeDef pitchmotor_speed_auto;
PidTypeDef yawmotor_speed_auto;
PidTypeDef pitchmotor_auto;

PidTypeDef triggermotor;



//初始化PID 
void PID_Init(PidTypeDef * pid)
{
	memset(pid, 0, sizeof(PidTypeDef));
}


void PID_InitALL(void)                //初始化位置环，速度环参数
{
	////底盘电机
	PID_Init(&chassismotor_1);
 	PID_SetParam(&chassismotor_1 , 50 , 0.5 , 10 , 2000 , 10000); //50
	PID_Init(&chassismotor_2);
 	PID_SetParam(&chassismotor_2 , 50 , 0.5 , 10 , 2000 , 10000);		            
	PID_Init(&chassismotor_3);
 	PID_SetParam(&chassismotor_3 , 50 , 0.5 , 10 , 2000 , 10000);		             
	PID_Init(&chassismotor_4);
 	PID_SetParam(&chassismotor_4 , 50 , 0.5 , 10 , 2000 , 10000);		
	

	////遥控模式
	PID_Init(&yawmotor_remote);
 	PID_SetParam(&yawmotor_remote , 0.01 , 0.0001 , 0 , 0.4 , 4);		
	PID_Init(&yawmotor_speed_remote);
 	PID_SetParam(&yawmotor_speed_remote , 1000 , 0.001 , 150 , 100 , 10000);	
	
	PID_Init(&pitchmotor_remote);
 	PID_SetParam(&pitchmotor_remote , 0.02 , 0.001 , 0 , 1 , 3);	
	PID_Init(&pitchmotor_speed_remote);
 	PID_SetParam(&pitchmotor_speed_remote , 1000 , 0.001 , 150 , 100 , 10000);	

 
	////自动跟踪
  //////稳定版本：
  PID_Init(&yawmotor_auto);
 	PID_SetParam(&yawmotor_auto , 0.005 , 0.00001 , 0 , 0.4 , 4);		
	PID_Init(&yawmotor_speed_auto);
 	PID_SetParam(&yawmotor_speed_auto , 1200 , 0.001 , 1000 , 100 , 10000);	
	
	PID_Init(&pitchmotor_auto);
 	PID_SetParam(&pitchmotor_auto , 0.003 , 0.00001 , 0 , 0.8 , 3);	//	0.003 , 0.0001 , 0 , 0.5 , 3
	PID_Init(&pitchmotor_speed_auto);
 	PID_SetParam(&pitchmotor_speed_auto , 1200 , 0.001 , 1000 , 100 , 10000);	


  ////拨弹电机
	PID_Init(&triggermotor);
 	PID_SetParam(&triggermotor , 1 , 0.011 , 0 , 5000 , 10000);		
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
	pid->out_kd = (pid->err - pid->last_err) * pid->Kd;
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
















