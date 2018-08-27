#include "chassis_task.h"
#include "pid.h"
#include "remote_task.h"
#include "can2.h"
#include "usart2.h" 

extern int rc_ch0_value , rc_ch1_value , rc_ch2_value , rc_ch3_value;
float gimbal_chassis_err = 0 , gimbal_chassis_para = 5 ;//50

//����ǣ�
int chassis_orientaion_targetangle = 0 , chassis_orientaion_targetangle_flag1 , chassis_orientaion_targetangle_flag2;

int chassis_turn_tail_flag = 0 , chassis_turn_tail_stopgap_flag = 0;

extern s16 Yaw_Motor_encoding;

//ǰ�������״�����룺
extern s16 front_distance_from_lidar;

void chassis_task(void)
{
//	angle_allocation();
	
	switch(RC_CtrlData.rc.s2)
	{
	  //stop �� ң��δ������		
		case STOP:
			Send_motorvalue_CAN1_4(0,0,0,0);
		break;
		
 		case NULL:
			Send_motorvalue_CAN1_4(0,0,0,0);
		break;
		
	  //ң��ģʽ
		case REMOTE_INPUT:
			remote_chassis_control_mode();
		break;
		
	  //�Զ�ģʽ
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
		//�����˶�����
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
		//�ϲ�ֱ�ӿ���x,y,z
		case 1:automatic_chassis_mode1();
			break;
		
		//�ϲ���ƹ̶��Ƕ�
		case 2:automatic_chassis_mode2();
			break;
		
		//���е�ģʽ
		case 3:automatic_chassis_mode3();
			break;	

		//��β���أ�
		case 4:automatic_chassis_mode4();
			break;			
	}  
 
}

/////////////////////////
void angle_allocation()
{
	//�������䣺		
	//��㣺
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
	//180�㣺
	if(chassis_orientaion_targetangle == 180)
	{
		chassis_orientaion_targetangle_flag1 = 0;
		chassis_orientaion_targetangle_flag2 = 0;
		gimbal_chassis_err =  (Uwb_data.yaw_angle_uwb - 180)* gimbal_chassis_para;
	}
	/////  0 - 180   180 - 360
	if(chassis_orientaion_targetangle > 0   && chassis_orientaion_targetangle < 180) //С����
	{
		chassis_orientaion_targetangle_flag1 = 1;
		chassis_orientaion_targetangle_flag2 = 0;
	}
	if(chassis_orientaion_targetangle > 180 && chassis_orientaion_targetangle < 360) //������
	{
		chassis_orientaion_targetangle_flag1 = 0;
		chassis_orientaion_targetangle_flag2 = 1;
	}
	////////////
	//С���� 0 - 180 
	if(chassis_orientaion_targetangle_flag1 == 1)
	{
		//��������
		if(Uwb_data.yaw_angle_uwb >= chassis_orientaion_targetangle && Uwb_data.yaw_angle_uwb < (chassis_orientaion_targetangle + 180)) 
		{
			gimbal_chassis_err   = (        Uwb_data.yaw_angle_uwb  - chassis_orientaion_targetangle) * gimbal_chassis_para;
		}
		//����������
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
	//������ 180 - 360
	if(chassis_orientaion_targetangle_flag2 == 1)
	{
		//��������
		if(Uwb_data.yaw_angle_uwb >= (chassis_orientaion_targetangle - 180) && Uwb_data.yaw_angle_uwb < chassis_orientaion_targetangle) 
		{
			gimbal_chassis_err   = (        Uwb_data.yaw_angle_uwb  - chassis_orientaion_targetangle) * gimbal_chassis_para;
		}
		//����������
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

	//�����˶�����
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
	gimbal_chassis_err  = (Uwb_data.yaw_angle_uwb - Top_Data.Top_velocity_yaw) * gimbal_chassis_para ; //���̸���

	//�����˶�����
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
 

	//�����˶�����
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
		//���̰�β
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
 


	//�����˶�����
	chassis_Motor_M1_set = -move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M2_set =  move_for_behind_value + move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M3_set =  move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
	chassis_Motor_M4_set = -move_for_behind_value - move_left_right_value + gimbal_chassis_err ; 
			
	Send_motorvalue_CAN1_4( PID_Calc(&chassismotor_1 , chassis_Motor_M1[1] , chassis_Motor_M1_set),  
													PID_Calc(&chassismotor_2 , chassis_Motor_M2[1] , chassis_Motor_M2_set),  
													PID_Calc(&chassismotor_3 , chassis_Motor_M3[1] , chassis_Motor_M3_set),  
													PID_Calc(&chassismotor_4 , chassis_Motor_M4[1] , chassis_Motor_M4_set));   
}
			