#include "chassis_task.h"
#include "pid.h"
#include "remote_task.h"
#include "can2.h"
#include "usart2.h" 

int abs(int a)
{
	if(a < 0)
	{
		a = -a;
	}
	return a;
}

extern int rc_ch0_value , rc_ch1_value , rc_ch2_value , rc_ch3_value;
float gimbal_chassis_err = 0 , gimbal_chassis_para = 5  ;//50

int chassis_turn_tail_flag = 0 , chassis_turn_tail_stopgap_flag = 0;

extern s16 Yaw_Motor_encoding;

//����ǣ�
int chassis_orientaion_targetangle = 0 , chassis_orientaion_targetangle_flag1 , chassis_orientaion_targetangle_flag2;

//ǰ�������״�����룺
extern s16 front_distance_from_lidar;

//��翪�ؼ�⣺
int guangdian_detect_flag1 = 0 , guangdian_detect_last_flag1 = 0 , guangdian_detect_flag2 = 0 , guangdian_detect_last_flag2 = 0;
//���е�status��
int grab_centerpoint_status = 0;

extern int grab_center_segmentflag;

extern uint16_t get_the_attacking_buff_flag ;
void chassis_task(void)
{
	guangdian_detect_flag1 = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2) ;// 1:��  �� 0����
	guangdian_detect_flag2 = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9) ;// 1:��  �� 0����
			
  //////////////////////////////////////
	//�Ƕȷ��䣺
//  angle_allocation();
	/////////////////////////////////////////////

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
	
	guangdian_detect_last_flag1 = guangdian_detect_flag1;
	guangdian_detect_last_flag2 = guangdian_detect_flag2;
}
 
/////////////////////////////////////////////////////////////////

void remote_chassis_control_mode()
{
//	gimbal_chassis_err = -(Yaw_Motor_encoding - Yaw_motorloco_init) * 0.1;
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

///////////////////////////////////////////////////
extern int game_start_flag;
int grab_center_FLAG_ONCE = 1;
void Automatic_chassis_control()
{
//	if(grab_center_FLAG_ONCE)
//	{
//	  Top_Data.Top_Chassis_Mode = 1;
//	}

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

///////////////////////////////////////////////////////////////////////////////////////

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


/////////////////////////////////////////////////////////////////////////////////////////////////////
void automatic_chassis_mode1()
{
	move_for_behind_value =  Top_Data.Top_velocity_x * 2 / 1.41421;
	move_left_right_value = -Top_Data.Top_velocity_y * 2 / 1.41421;
	gimbal_chassis_err    = -Top_Data.Top_velocity_yaw;
				
	 /////////////////
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
		
	 /////////////////
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
  if(grab_centerpoint_status == 0)
	{
		move_for_behind_value = 100;
		move_left_right_value = 0;
	}
	if(grab_centerpoint_status == 0 && guangdian_detect_flag2 == 0 && guangdian_detect_last_flag2 == 1)
	{
		grab_centerpoint_status = 1;
	}
	if(grab_centerpoint_status == 1)
	{
		move_for_behind_value = -10;
		move_left_right_value = -100;		
	}
	if(grab_centerpoint_status == 1 && guangdian_detect_flag2 == 1 && guangdian_detect_last_flag2 == 0)
	{
		grab_centerpoint_status = 2;
	}
	if(grab_centerpoint_status == 2)
	{
		move_for_behind_value = 0;
		move_left_right_value = 0;		
		get_the_attacking_buff_flag = 1;
		grab_center_FLAG_ONCE = 0;
	}
		
	///////////////////////////////////////////////////////
//	//���е㣺
//	//��һ�Σ�
//	if(grab_centerpoint_status == 0)
//	{
//		move_for_behind_value = grab_centerpoint_speedvalue;
//		chassis_orientaion_targetangle = 0;
//	}
//	if(grab_centerpoint_status == 0 && guangdian_detect_flag1 == 1 && guangdian_detect_last_flag1 == 0)
//	{
//		grab_centerpoint_status = 1;
//		move_for_behind_value = 0;// grab_centerpoint_speedvalue;
//		move_left_right_value = 0;//-grab_centerpoint_speedvalue;
//		chassis_orientaion_targetangle = 90;
//	} 
//	
//	//�ڶ��Σ�
//	if(grab_centerpoint_status == 1 && (Uwb_data.yaw_angle_uwb - chassis_orientaion_targetangle > 0))
//	{
//		grab_centerpoint_status = 2;
//	}
//	//
//	if(grab_centerpoint_status == 2 && (chassis_orientaion_targetangle - Uwb_data.yaw_angle_uwb > 0))
//	{
//		grab_centerpoint_status = 3;
//	}
//	//
//	if(grab_centerpoint_status == 3 && (Uwb_data.yaw_angle_uwb - chassis_orientaion_targetangle > 0))
//	{
//		grab_centerpoint_status = 4;
//	}
// 
//	if(grab_centerpoint_status == 4)
//	{
//		move_for_behind_value = grab_centerpoint_speedvalue;
//		move_left_right_value = 0;
//		chassis_orientaion_targetangle = 90;
//	}
//	
//	//�����Σ�
//	if(grab_centerpoint_status == 4 && guangdian_detect_flag2 == 1 && guangdian_detect_last_flag2 == 0)
//	{
//		grab_centerpoint_status = 5;
//		move_for_behind_value = grab_centerpoint_speedvalue ;
//		move_left_right_value = 0;
//		chassis_orientaion_targetangle = 0;
//	}
//	if(grab_centerpoint_status == 5 && Uwb_data.x_uwb >= 300)
//	{
//		grab_centerpoint_status = 6;
//	}
//	
//	//�е㣺
//	if(grab_centerpoint_status == 6)
//	{
//		move_for_behind_value = 2 * (400 - Uwb_data.x_uwb);
//		move_left_right_value =-2 * (250 - Uwb_data.y_uwb);
//	}
//	
//	//��λ�ò���β:
//	if(Uwb_data.x_uwb >= 400)
//	{
//		grab_centerpoint_status = 7;
//	}
//	if(grab_centerpoint_status == 7)		
//	{
//		move_for_behind_value = 2 * (400 - Uwb_data.x_uwb);
//		move_left_right_value =-2 * (250 - Uwb_data.y_uwb);
//		
//		//���̰�β
//		if(chassis_turn_tail_flag == 0)
//		{
//			gimbal_chassis_err = -50;
//			if(Yaw_Motor_encoding <= (7600 - 2*(7600 - Yaw_motorloco_init)) )
//			{
//				chassis_turn_tail_flag = 1;
//			}
//		}
//		if(chassis_turn_tail_flag == 1)
//		{
//			gimbal_chassis_err = 50;
//			if(Yaw_Motor_encoding >= 7600)
//			{
//				chassis_turn_tail_flag = 0;
//			}
//		}
//	}

	/////////////////
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
		if(Yaw_Motor_encoding <= (7600 - 2*(7600 - Yaw_motorloco_init)) )
		{
			chassis_turn_tail_flag = 1;
		}
	}
	if(chassis_turn_tail_flag == 1)
	{
		gimbal_chassis_err = 50;
		if(Yaw_Motor_encoding >= 7600)
		{
			chassis_turn_tail_flag = 0;
		}
	}
	
//		  //���̰�β
//			if(chassis_turn_tail_flag == 0 && chassis_turn_tail_stopgap_flag == 0)
//			{
//				gimbal_chassis_err = -50;
//				if(Yaw_Motor_encoding <= (7600 - 2*(7600 - Yaw_motorloco_init)) )
//				{
//					chassis_turn_tail_flag = 1;
//					chassis_turn_tail_stopgap_flag = 1;
//				}
//			}
//			if(chassis_turn_tail_flag == 1 && chassis_turn_tail_stopgap_flag == 0)
//			{
//				gimbal_chassis_err = 50;
//				if(Yaw_Motor_encoding >= 7600)
//				{
//					chassis_turn_tail_flag = 0;
//					chassis_turn_tail_stopgap_flag = 1;
//				}
//			}
//			if(chassis_turn_tail_stopgap_flag == 1)
//			{
//				gimbal_chassis_err = 0;
//			}

	 /////////////////
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





















	/////
//		if(grab_centerpoint_status == 0)
//		{
//	   	move_for_behind_value = grab_centerpoint_speedvalue;
//			chassis_orientaion_targetangle = 0;
//		}
//		if(grab_centerpoint_status == 0 && guangdian_detect_flag1 == 1 && guangdian_detect_last_flag1 == 0)
//		{
//			grab_centerpoint_status = 1;
//			move_for_behind_value = 0;// grab_centerpoint_speedvalue;
//			move_left_right_value = -grab_centerpoint_speedvalue;//-grab_centerpoint_speedvalue;
//			chassis_orientaion_targetangle = 90;
//		}
//		
//    //�ڶ��Σ�
//		if(grab_centerpoint_status == 1 && abs(chassis_orientaion_targetangle - Uwb_data.yaw_angle_uwb) < 10)
//		{
//			grab_centerpoint_status = 2;
//		}
//		if(grab_centerpoint_status == 2)
//		{
//	   	move_for_behind_value = grab_centerpoint_speedvalue + 200;
//			move_left_right_value = 0;
//			chassis_orientaion_targetangle = 90;
//		}
//		
//    //�����Σ�
//		if(grab_centerpoint_status == 2 && guangdian_detect_flag2 == 1 && guangdian_detect_last_flag2 == 0)
//		{
//			grab_centerpoint_status = 3;
//	   	move_for_behind_value = grab_centerpoint_speedvalue + 200;
//			move_left_right_value = 0;
//			chassis_orientaion_targetangle = 0;
//		}
//   
//		 if(grab_centerpoint_status == 4)
//		 {
//			 move_for_behind_value = 0;
//			 move_left_right_value = 0;
//			 chassis_orientaion_targetangle = 0;
//		 }
//		
