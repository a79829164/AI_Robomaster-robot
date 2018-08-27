#include "chassis_task.h"
#include "pid.h"
#include "remote_task.h"
#include "can2.h"
#include "usart2.h" 

extern int rc_ch0_value , rc_ch1_value , rc_ch2_value , rc_ch3_value;
float gimbal_chassis_err = 0 , gimbal_chassis_para = 5 ;//50

//方向角：
int chassis_orientaion_targetangle = 0 , chassis_orientaion_targetangle_flag1 , chassis_orientaion_targetangle_flag2;

int chassis_turn_tail_flag = 0 , chassis_turn_tail_stopgap_flag = 0;

extern s16 Yaw_Motor_encoding;

//前方激光雷达测距距离：
extern s16 front_distance_from_lidar;

void chassis_task(void)
{
//	angle_allocation();
	
	switch(RC_CtrlData.rc.s2)
	{
	  //stop 或 遥控未开启：		
		case STOP:
			Send_motorvalue_CAN1_4(0,0,0,0);
		break;
		
 		case NULL:
			Send_motorvalue_CAN1_4(0,0,0,0);
		break;
		
	  //遥控模式
		case REMOTE_INPUT:
			remote_chassis_control_mode();
		break;
		
	  //自动模式
		case AUTOMATIC_INPUT:
			Automatic_chassis_control();
		break;
	}
 
	speed_x = 0.01 * (-chassis_Motor_M1[1] + chassis_Motor_M3[1] + chassis_Motor_M2[1] - chassis_Motor_M4[1]) / 4.0 * 1.41421 / 2.0 ; 
	speed_y = 0.01 * (-chassis_Motor_M1[1] + chassis_Motor_M3[1] - chassis_Motor_M2[1] + chassis_Motor_M4[1]) / 4.0 * 1.41421 / 2.0 ;
}

/////////////////////////////////////////////////////////////////
void remote_chassis_control_mode()
{
		//底盘运动解算
	chassis_Motor_M1_set = -rc_ch1_value + rc_ch0_value + gimbal_chassis_err ; 
	chassis_Motor_M2_set =  rc_ch1_value + rc_ch0_value + gimbal_chassis_err ; 
	chassis_Motor_M3_set =  rc_ch1_value - rc_ch0_value + gimbal_chassis_err ; 
	chassis_Motor_M4_set = -rc_ch1_value - rc_ch0_value + gimbal_chassis_err ; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));  
}

/////////////////////////////////////
void Automatic_chassis_control()
{
//		Top_Data.Top_Chassis_Mode = 4;
 
	switch(Top_Data.Top_Chassis_Mode)
	{
		//上层直接控制x,y,z
		case 1:automatic_chassis_mode1();
			break;
		
		//上层控制固定角度
		case 2:automatic_chassis_mode2();
			break;
		
		//抢中点模式
		case 3:automatic_chassis_mode3();
			break;	

		//摆尾防守：
		case 4:automatic_chassis_mode4();
			break;			
	}  
 
}

/////////////////////////
void angle_allocation()
{
	//分配区间：		
	//零点：
	if(chassis_orientaion_targetangle == 0)
	{
		chassis_orientaion_targetangle_flag1 = 0;
		chassis_orientaion_targetangle_flag2 = 0;
		
		if(Uwb_data.yaw_angle_uwb >= 0 && Uwb_data.yaw_angle_uwb < 180)
		{
			gimbal_chassis_err =         Uwb_data.yaw_angle_uwb * gimbal_chassis_para;
		}
		if(Uwb_data.yaw_angle_uwb > 180)
		{
			gimbal_chassis_err = -(360 - Uwb_data.yaw_angle_uwb)* gimbal_chassis_para;
		}
	}
	//180°：
	if(chassis_orientaion_targetangle == 180)
	{
		chassis_orientaion_targetangle_flag1 = 0;
		chassis_orientaion_targetangle_flag2 = 0;
		gimbal_chassis_err =  (Uwb_data.yaw_angle_uwb - 180)* gimbal_chassis_para;
	}
	/////  0 - 180   180 - 360
	if(chassis_orientaion_targetangle > 0   && chassis_orientaion_targetangle < 180) //小区间
	{
		chassis_orientaion_targetangle_flag1 = 1;
		chassis_orientaion_targetangle_flag2 = 0;
	}
	if(chassis_orientaion_targetangle > 180 && chassis_orientaion_targetangle < 360) //大区间
	{
		chassis_orientaion_targetangle_flag1 = 0;
		chassis_orientaion_targetangle_flag2 = 1;
	}
	////////////
	//小区间 0 - 180 
	if(chassis_orientaion_targetangle_flag1 == 1)
	{
		//连续区间
		if(Uwb_data.yaw_angle_uwb >= chassis_orientaion_targetangle && Uwb_data.yaw_angle_uwb < (chassis_orientaion_targetangle + 180)) 
		{
			gimbal_chassis_err   = (        Uwb_data.yaw_angle_uwb  - chassis_orientaion_targetangle) * gimbal_chassis_para;
		}
		//不连续区间
		else
		{
			if(Uwb_data.yaw_angle_uwb >= 0 && Uwb_data.yaw_angle_uwb < chassis_orientaion_targetangle)
			{
				gimbal_chassis_err = (        Uwb_data.yaw_angle_uwb  - chassis_orientaion_targetangle) * gimbal_chassis_para;
			}
			else
			{
				gimbal_chassis_err =-( (360 - Uwb_data.yaw_angle_uwb) + chassis_orientaion_targetangle) * gimbal_chassis_para;
			}
		}
	}
	//大区间 180 - 360
	if(chassis_orientaion_targetangle_flag2 == 1)
	{
		//连续区间
		if(Uwb_data.yaw_angle_uwb >= (chassis_orientaion_targetangle - 180) && Uwb_data.yaw_angle_uwb < chassis_orientaion_targetangle) 
		{
			gimbal_chassis_err   = (        Uwb_data.yaw_angle_uwb  - chassis_orientaion_targetangle) * gimbal_chassis_para;
		}
		//不连续区间
		else
		{
			if(Uwb_data.yaw_angle_uwb >= 0 && Uwb_data.yaw_angle_uwb < (chassis_orientaion_targetangle - 180))
			{
				gimbal_chassis_err = (  360 + Uwb_data.yaw_angle_uwb  +-chassis_orientaion_targetangle) * gimbal_chassis_para;
			}
			else
			{
				gimbal_chassis_err = (        Uwb_data.yaw_angle_uwb  - chassis_orientaion_targetangle) * gimbal_chassis_para;
			}
		}
	}	
}
	
void automatic_chassis_mode1()
{
	move_for_behind_value =  Top_Data.Top_velocity_x * 2 / 1.41421  ;
	move_left_right_value = -Top_Data.Top_velocity_y * 2 / 1.41421  ;
	gimbal_chassis_err    = -Top_Data.Top_velocity_yaw;

	//底盘运动解算
	chassis_Motor_M1_set = -move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M2_set =  move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M3_set =  move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M4_set = -move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));   
}
	
void automatic_chassis_mode2()
{
	move_for_behind_value =  Top_Data.Top_velocity_x * 2 / 1.41421;
	move_left_right_value = -Top_Data.Top_velocity_y * 2 / 1.41421;
	gimbal_chassis_err  = (Uwb_data.yaw_angle_uwb - Top_Data.Top_velocity_yaw) * gimbal_chassis_para ; //底盘跟随

	//底盘运动解算
	chassis_Motor_M1_set = -move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M2_set =  move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M3_set =  move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M4_set = -move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));   
}

void automatic_chassis_mode3()
{
 

	//底盘运动解算
	chassis_Motor_M1_set = -move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M2_set =  move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M3_set =  move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M4_set = -move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));   
}

void automatic_chassis_mode4()
{
		//底盘摆尾
	if(chassis_turn_tail_flag == 0)
	{
		gimbal_chassis_err = -50;
		if(Yaw_Motor_encoding <= 5750 )
		{
			chassis_turn_tail_flag = 1;
		}
	}
	if(chassis_turn_tail_flag == 1)
	{
		gimbal_chassis_err = 50;
		if(Yaw_Motor_encoding >= 2 * Yaw_motorloco_init - 5750)
		{
			chassis_turn_tail_flag = 0;
		}
	}
 


	//底盘运动解算
	chassis_Motor_M1_set = -move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M2_set =  move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M3_set =  move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M4_set = -move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));   
}
			