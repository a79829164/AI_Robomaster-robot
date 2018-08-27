#include "chassis_task.h"
#include "pid.h"
#include "remote_task.h"


extern int rc_ch0_value , rc_ch1_value , rc_ch2_value , rc_ch3_value;
float gimbal_chassis_err = 0 , gimbal_chassis_para = -0.75;

float rotate_err = 0 , rotate_para = 150;
void chassis_task(void)
{
//	gimbal_chassis_err   = (Yaw_Motor[0] - Yaw_motorloco_init) * gimbal_chassis_para; //底盘跟随
//				
//	//如果stop，则全部静止
//	if(RC_CtrlData.rc.s2 == 2)
//	{
//		gimbal_chassis_err = 0;
//	}
//	
//	if(RC_CtrlData.rc.s2 != 2)
//	{
//		//底盘运动解算
//		chassis_Motor_M1_set = -rc_ch1_value + rc_ch0_value + gimbal_chassis_err ; 
//		chassis_Motor_M2_set =  rc_ch1_value + rc_ch0_value + gimbal_chassis_err ; 
//		chassis_Motor_M3_set =  rc_ch1_value - rc_ch0_value + gimbal_chassis_err ; 
//		chassis_Motor_M4_set = -rc_ch1_value - rc_ch0_value + gimbal_chassis_err ; 
//	}
	rotate_err = (rc_ch2_value * 0.01 - gimbal_speed_yaw) * rotate_para;
	if(RC_CtrlData.rc.s2 != 2)
	{
		//底盘运动解算
		chassis_Motor_M1_set = -rc_ch1_value + rc_ch0_value + rotate_err ; 
		chassis_Motor_M2_set =  rc_ch1_value + rc_ch0_value + rotate_err ; 
		chassis_Motor_M3_set =  rc_ch1_value - rc_ch0_value + rotate_err ; 
		chassis_Motor_M4_set = -rc_ch1_value - rc_ch0_value + rotate_err ; 
	}
	
	Send_motorvalue_CAN1_4(PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
	                       PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
	                       PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
	                       PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));  
  
}



 

