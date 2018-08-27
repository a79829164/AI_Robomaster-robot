#include "gimbal_task.h"
#include "pid.h"
#include "remote_task.h"
#include "usart2.h" 


int f(int a);

int automaticmode_notarget_cnt;
float yaw_locationPID_input , pitch_locationPID_input_last , yaw_locationPID_input_updateFlag = 1;
float pitch_locationPID_input;

extern u8 Set_Navipoint_status;

//�Զ���̨ģʽ��3
int from_realsense_angle_flag = 1;

//��һ����̨ģʽ:
int Top_Data_last_Top_Gimbal_Mode = 0;
int searching_start_flag = 0;
float searching_standard_position = 0;

//��̨Ԥ��̶�ƫת��
extern float fixed_turn_angle_value ;

s16 Yaw_Motor_encoding;

extern uint16_t Bullet_freq_tar;

int shake_gimbal_once_flag = 0;

extern int shoot_flag;
void gimbal_task(void)
{
	//û���ҵ�Ŀ�꣬ҡͷ,�����־��
	if(Top_Data.Top_Gimbal_Mode != 1)
	{
		shake_gimbal_once_flag = 0;
	}
	
	//û���ҵ�Ŀ�꣬ҡͷ
	if(Top_Data.Top_Gimbal_Mode != 3)
	{
		from_realsense_angle_flag = 1;
	}
	
	//ת��yaw����λ�ã�
	Yaw_Motor_encoding = Yaw_Motor[0];
	if(Yaw_Motor_encoding <= 2000)//�Ѿ�Խ��
	{	
		Yaw_Motor_encoding = Yaw_Motor_encoding + 8191;
	}
	
	//�������㣺
	if(Trigger_Motor_Set == 0)
	{
		triggermotor.out_ki = 0;
	}
	
  ////////////////////////////////////////////////////////////////////////
	
	switch(RC_CtrlData.rc.s2)
	{
    //stop �� ң��δ������		
		case STOP:
			Trigger_Motor_Set = 0;		
			if(Trigger_Motor_Set == 0)
			{
				triggermotor.out_ki = 0;
			}
			Send_motorvalue_CAN1_3(0,0,0,0);
		break;
		
 		case NULL:
			Trigger_Motor_Set = 0;		
			if(Trigger_Motor_Set == 0)
			{
				triggermotor.out_ki = 0;
			}
			Send_motorvalue_CAN1_3(0,0,0,0);
		break;
			
		//ң��ģʽ
		case REMOTE_INPUT:
			remote_gimbal_control_mode();
		break;
		
	  //�Զ�ģʽ
		case AUTOMATIC_INPUT:
			
			if(Top_Data.Top_Gimbal_Mode == 0 || Top_Data.Top_Gimbal_Mode == 1 || Top_Data.Top_Gimbal_Mode == 2 || Top_Data.Top_Gimbal_Mode == 3 || Top_Data.Top_Gimbal_Mode == 4 || Top_Data.Top_Gimbal_Mode == 5)
			{
		    Trigger_Motor_Set = 0;
			}
	
			Automatic_gimbal_control();
		break;
	}

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
void remote_gimbal_control_mode()
{
	if(Trigger_Motor_Set == 0)
	{
		triggermotor.out_ki = 0;
	}
	
	yaw_locationPID_input = -yaw_angle_set  + Yaw_motorloco_init;//6367
	if(yaw_locationPID_input >= 8190)
	{
		yaw_locationPID_input = 8190;
	}
	if( yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)) )
	{
		yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
	}
	
	gimbal_speed_yaw_set   =    PID_Calc(&yawmotor_remote         , Yaw_Motor_encoding , yaw_locationPID_input);   
	
	gimbal_speed_pitch_set =    PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     , pitch_angle_set + Pitch_motorloco_init);   
	
	Send_motorvalue_CAN1_3(    -PID_Calc(&yawmotor_speed_remote   , gimbal_speed_yaw   , gimbal_speed_yaw_set),  //
															PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor            , Trigger_Motor[1]   , Trigger_Motor_Set), 
															0); 
}

/////////////////////////////////////////////
extern int game_start_flag;

void Automatic_gimbal_control()
{
//	Top_Data.Top_Gimbal_Mode = 1;
	switch(Top_Data.Top_Gimbal_Mode)
	{
		case 0:automatic_gimbal_mode0();
		  break;
		
		//ҡͷ��
		case 1:automatic_gimbal_mode1();
			break;
		
		//realsense����
		case 3:automatic_gimbal_mode3();
			break;				
		//realsense����
		case 5:automatic_gimbal_mode5();
			break;

		//��̨camera���ƣ�
		case 2:automatic_gimbal_mode2();
			break;		
		case 4:automatic_gimbal_mode4();
			break;			
		//���������
		case 6:automatic_gimbal_mode6();
			break;	
		//���������
		case 7:automatic_gimbal_mode7();
			break;
		//���������
		case 8:automatic_gimbal_mode8();
			break;		
	}  	

}

/////////////////////////////////////////////////////////////////////////////////////////

	
void automatic_gimbal_mode0()
{
	Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_remote   , gimbal_speed_yaw   , 0),  
														PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -0), 
														PID_Calc(&triggermotor            , Trigger_Motor[1]   , Trigger_Motor_Set), 
														0); 	
}



//ҡͷ��
void automatic_gimbal_mode1()
{
	if(shake_gimbal_once_flag == 0)
	{
		yaw_locationPID_input = Yaw_Motor_encoding;
		shake_gimbal_once_flag = 1;
	}
	//��ת��ѰĿ�꣺
	if(Search_enemy_turning_flag == 0)
	{
		yaw_locationPID_input = yaw_locationPID_input + 1.4;
		pitch_locationPID_input = Pitch_motorloco_init;
	}
	if(Search_enemy_turning_flag == 1)
	{
		yaw_locationPID_input = yaw_locationPID_input - 1.4;
		pitch_locationPID_input = Pitch_motorloco_init - 300;
	}

	//��̨�����λ��		
	if(yaw_locationPID_input >= 8190)
	{
		yaw_locationPID_input = 8190;
		Search_enemy_turning_flag = 1;
	}
	if( yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)) )
	{
		yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
		Search_enemy_turning_flag = 0;
	}
	
	gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_remote         , Yaw_Motor_encoding , yaw_locationPID_input);  
	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     , pitch_locationPID_input);   
	
	Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_remote   , gimbal_speed_yaw   , gimbal_speed_yaw_set),  
														PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
														PID_Calc(&triggermotor            , Trigger_Motor[1]   , Trigger_Motor_Set), 
														0); 
}

//realsense:
//case3: realsense����Ŀ�꣬��̨����
void automatic_gimbal_mode3()
{
  //�ǶȽ��㣺
	Yaw_motor_realloco     =  Yaw_Motor_encoding;
	Pitch_motor_realloco   =  Pitch_Motor[0];

	yaw_angle_dirta        = -top_Yaw   / 100.0 / 360.0 * 8192;
	pitch_angle_dirta      = -top_Pitch / 100.0 / 360.0 * 8192;	
	
	if(abs(top_Yaw) > 10 && from_realsense_angle_flag == 1)
	{
		yaw_locationPID_input  = yaw_angle_dirta + Yaw_motor_realloco;//6400
		from_realsense_angle_flag = 0;
	}
	
	//��̨�����λ��
	if(yaw_locationPID_input >= 8190)
	{
		yaw_locationPID_input = 8190;
	}
	if(yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)))
	{
		yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
	}
	
	//���ݾ���̧ͷ
	if(top_distance < 0.7)
	{
		pitch_locationPID_input = Pitch_motorloco_init - 400;
	}
	else
	{
		pitch_locationPID_input = Pitch_motorloco_init;
	}
	
	//PID���㣺
	gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_auto   , Yaw_Motor_encoding , yaw_locationPID_input);  
	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_auto , Pitch_Motor[0]     , pitch_locationPID_input);  //pitch_angle_dirta + Pitch_motor_realloco
	
	Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto   , gimbal_speed_yaw   , gimbal_speed_yaw_set), 
														PID_Calc(&pitchmotor_speed_auto , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
														PID_Calc(&triggermotor          , Trigger_Motor[1]   , Trigger_Motor_Set), //
														0); 
}

void automatic_gimbal_mode4()
{
//			from_realsense_angle_flag = 1;
//			
//			//��֡�������㣺
//			if(top_Yaw == 0 && (top_Pitch == 0 || top_Pitch == 32760))
//			{
//				yawmotor_auto.out_ki = 0;
//			}

//			//�ǶȽ��㣺
//			Yaw_motor_realloco     =  Yaw_Motor_encoding;
//			Pitch_motor_realloco   =  Pitch_Motor[0];

//			yaw_angle_dirta        = -top_Yaw   / 100.0 / 360.0 * 8192;
//			pitch_angle_dirta      = -top_Pitch / 100.0 / 360.0 * 8192;

//			
////			fixed_turn_angle_value = 
////			if(yaw_angle_dirta > 5)
////			{
////				yaw_angle_dirta = yaw_angle_dirta + fixed_turn_angle_value;
////			}
////			if(yaw_angle_dirta < -5)
////			{
////				yaw_angle_dirta = yaw_angle_dirta - fixed_turn_angle_value;
////			}
//			
//			
//	    yaw_locationPID_input  = yaw_angle_dirta + Yaw_motor_realloco;//6400
//			
//	  	//��̨�����λ��
//			if(yaw_locationPID_input >= 8190)
//			{
//				yaw_locationPID_input = 8190;
//			}
//			if(yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)))
//			{
//				yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
//			}

//			//PID���㣺
//			gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_auto   , Yaw_Motor_encoding     , yaw_locationPID_input);  
//			gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_auto , Pitch_Motor[0]   , pitch_angle_dirta + Pitch_motor_realloco);  
//				
//			//û�м�⵽װ�װ�:
//			if(top_Pitch == 32760)
//			{	
//	    	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     ,  Pitch_motorloco_init);   
//			
//				Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto     , gimbal_speed_yaw   ,  gimbal_speed_yaw_set),  
//																	PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
//																	PID_Calc(&triggermotor            , Trigger_Motor[1]   ,  Trigger_Motor_Set), 
//																	0); 
//			}
//			//�м�⵽װ�װ�:
//			else
//			{
//				Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto   , gimbal_speed_yaw   , gimbal_speed_yaw_set), 
//																	PID_Calc(&pitchmotor_speed_auto , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
//																	PID_Calc(&triggermotor          , Trigger_Motor[1]   , Trigger_Motor_Set), //
//																	0); 
//			}
}

// ���е㣬realsense��� + �˶�
void automatic_gimbal_mode5()
{
		//��֡�������㣺
	if(top_Yaw == 0 && (top_Pitch == 0 || top_Pitch == 32760))
	{
		yawmotor_auto.out_ki = 0;
	}

	//�ǶȽ��㣺
	Yaw_motor_realloco     =  Yaw_Motor_encoding;
	Pitch_motor_realloco   =  Pitch_Motor[0];

	yaw_angle_dirta        = -top_Yaw   / 100.0 / 360.0 * 8192;
	pitch_angle_dirta      = -top_Pitch / 100.0 / 360.0 * 8192;
	
	if(top_Pitch != 32760 && from_realsense_angle_flag == 1)
	{	
		yaw_locationPID_input  = yaw_angle_dirta + Yaw_motor_realloco;//6400
		from_realsense_angle_flag = 0;
	}
	
	//��̨�����λ��
	if(yaw_locationPID_input >= 8190)
	{
		yaw_locationPID_input = 8190;
	}
	if(yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)))
	{
		yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
	}

	//PID���㣺
	if(from_realsense_angle_flag == 0)
	{
		gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_auto   , Yaw_Motor_encoding , yaw_locationPID_input);  
		gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_auto , Pitch_Motor[0]     , Pitch_motorloco_init);  
		
		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto   , gimbal_speed_yaw   , gimbal_speed_yaw_set), 
															PID_Calc(&pitchmotor_speed_auto , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor          , Trigger_Motor[1]   , Trigger_Motor_Set), //
															0); 
	}
}

//��̨�����
//case2:�ҵ�Ŀ�꣬�Զ�����
void automatic_gimbal_mode2()
{
	from_realsense_angle_flag = 1;
	
	//��֡�������㣺
	if(top_Yaw == 0 && (top_Pitch == 0 || top_Pitch == 32760))
	{
		yawmotor_auto.out_ki = 0;
	}

	//�ǶȽ��㣺
	Yaw_motor_realloco     =  Yaw_Motor_encoding;
	yaw_angle_dirta        = -top_Yaw   / 100.0 / 360.0 * 8192;
	
	Pitch_motor_realloco   =  Pitch_Motor[0];
	pitch_angle_dirta      = -top_Pitch / 100.0 / 360.0 * 8192;
 
	yaw_locationPID_input   = yaw_angle_dirta + Yaw_motor_realloco;//6400
	pitch_locationPID_input = pitch_angle_dirta + Pitch_motor_realloco;
	
	//��̨�����λ��
	if(yaw_locationPID_input >= 8190)
	{
		yaw_locationPID_input = 8190;
	}
	if(yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)))
	{
		yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
	}
	
	//���ݾ���̧ͷ
	if(top_distance < 0.7)
	{
		pitch_locationPID_input = Pitch_motorloco_init - 400;
	}
	else
	{
		pitch_locationPID_input = Pitch_motorloco_init;
	}
	
	//PIDλ�ü��㣺
	gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_auto   , Yaw_Motor_encoding , yaw_locationPID_input);  
	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_auto , Pitch_Motor[0]     , pitch_locationPID_input);  
 
	//û�м�⵽װ�װ�:
	if(top_Pitch == 32760)
	{	
		gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     ,  pitch_locationPID_input);   
	
		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto     , gimbal_speed_yaw   ,  gimbal_speed_yaw_set),  
															PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor            , Trigger_Motor[1]   ,  Trigger_Motor_Set), 
															0); 
	}
//	//û�м�⵽װ�װ�:
//	else if(top_Pitch == 32761)
//	{	
//		gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     ,  pitch_locationPID_input_last);   
//	
//		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto     , gimbal_speed_yaw   ,  gimbal_speed_yaw_set),  
//															PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
//															PID_Calc(&triggermotor            , Trigger_Motor[1]   ,  Trigger_Motor_Set), 
//															0); 
//	}
	//�м�⵽װ�װ�:
	else// if(top_Pitch != 32760 && top_Pitch != 32761)
	{
		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto   , gimbal_speed_yaw   , gimbal_speed_yaw_set), 
															PID_Calc(&pitchmotor_speed_auto , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor          , Trigger_Motor[1]   , Trigger_Motor_Set), //
															0); 
		
	
	}
}	

// �ҵ�Ŀ����������
void automatic_gimbal_mode6()
{
	shoot_flag = 1;
	Trigger_Motor_Set = 2600;
	
	from_realsense_angle_flag = 1;
	
	//��֡�������㣺
	if(top_Yaw == 0 && (top_Pitch == 0 || top_Pitch == 32760))
	{
		yawmotor_auto.out_ki = 0;
	}

	//�ǶȽ��㣺
	Yaw_motor_realloco     =  Yaw_Motor_encoding;
	Pitch_motor_realloco   =  Pitch_Motor[0];

	yaw_angle_dirta        = -top_Yaw   / 100.0 / 360.0 * 8192;
	pitch_angle_dirta      = -top_Pitch / 100.0 / 360.0 * 8192;
	
	yaw_locationPID_input  = yaw_angle_dirta + Yaw_motor_realloco;//6400
	pitch_locationPID_input = pitch_angle_dirta + Pitch_motor_realloco;
	
	//��̨�����λ��
	if(yaw_locationPID_input >= 8190)
	{
		yaw_locationPID_input = 8190;
	}
	if(yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)))
	{
		yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
	}

	//PID���㣺
	gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_auto   , Yaw_Motor_encoding , yaw_locationPID_input);  
	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_auto , Pitch_Motor[0]     , pitch_locationPID_input);  
		
	//û�м�⵽װ�װ�:
	if(top_Pitch == 32760)
	{	
		gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     ,  Pitch_motorloco_init);   
	
		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto     , gimbal_speed_yaw   ,  gimbal_speed_yaw_set),  
															PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor            , Trigger_Motor[1]   ,  Trigger_Motor_Set), 
															0); 
	}
	//�м�⵽װ�װ�:
	else
	{
		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto   , gimbal_speed_yaw   , gimbal_speed_yaw_set), 
															PID_Calc(&pitchmotor_speed_auto , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor          , Trigger_Motor[1]   , Trigger_Motor_Set), //
															0); 
	}
}

// �ҵ�Ŀ���������
void automatic_gimbal_mode7()
{
	shoot_flag = 1;
	Trigger_Motor_Set = 2600;
	
	from_realsense_angle_flag = 1;
	
	//��֡�������㣺
	if(top_Yaw == 0 && (top_Pitch == 0 || top_Pitch == 32760))
	{
		yawmotor_auto.out_ki = 0;
	}

	//�ǶȽ��㣺
	Yaw_motor_realloco     =  Yaw_Motor_encoding;
	Pitch_motor_realloco   =  Pitch_Motor[0];

	yaw_angle_dirta        = -top_Yaw   / 100.0 / 360.0 * 8192;
	pitch_angle_dirta      = -top_Pitch / 100.0 / 360.0 * 8192;
 
	yaw_locationPID_input  = yaw_angle_dirta + Yaw_motor_realloco;//6400
	pitch_locationPID_input = pitch_angle_dirta + Pitch_motor_realloco;
		
	//��̨�����λ��
	if(yaw_locationPID_input >= 8190)
	{
		yaw_locationPID_input = 8190;
	}
	if(yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)))
	{
		yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
	}

	//PID���㣺
	gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_auto   , Yaw_Motor_encoding , yaw_locationPID_input);  
	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_auto , Pitch_Motor[0]     , pitch_locationPID_input);  
		
	//û�м�⵽װ�װ�:
	if(top_Pitch == 32760)
	{	
		gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     ,  Pitch_motorloco_init);   
	
		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto     , gimbal_speed_yaw   ,  gimbal_speed_yaw_set),  
															PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor            , Trigger_Motor[1]   ,  Trigger_Motor_Set), 
															0); 
	}
	//�м�⵽װ�װ�:
	else
	{
		Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto   , gimbal_speed_yaw   , gimbal_speed_yaw_set), 
															PID_Calc(&pitchmotor_speed_auto , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
															PID_Calc(&triggermotor          , Trigger_Motor[1]   , Trigger_Motor_Set), //
															0); 
	}
}

		
// �ҵ�Ŀ��, ���������
void automatic_gimbal_mode8()
{
	shoot_flag = 1;
	Trigger_Motor_Set = 2600;
	
	from_realsense_angle_flag = 1;

	//��֡�������㣺
	if(top_Yaw == 0 && (top_Pitch == 0 || top_Pitch == 32760))
	{
	yawmotor_auto.out_ki = 0;
	}

	//�ǶȽ��㣺
	Yaw_motor_realloco     =  Yaw_Motor_encoding;
	Pitch_motor_realloco   =  Pitch_Motor[0];

	yaw_angle_dirta        = -top_Yaw   / 100.0 / 360.0 * 8192;
	pitch_angle_dirta      = -top_Pitch / 100.0 / 360.0 * 8192;
 
	yaw_locationPID_input  = yaw_angle_dirta + Yaw_motor_realloco;//6400
	pitch_locationPID_input = pitch_angle_dirta + Pitch_motor_realloco;
		
	//��̨�����λ��
	if(yaw_locationPID_input >= 8190)
	{
	yaw_locationPID_input = 8190;
	}
	if(yaw_locationPID_input <= (8190 - 2*(8190 - Yaw_motorloco_init)))
	{
	yaw_locationPID_input = 8190 - 2*(8190 - Yaw_motorloco_init);
	}

	//PID���㣺
	gimbal_speed_yaw_set   =  PID_Calc(&yawmotor_auto   , Yaw_Motor_encoding , yaw_locationPID_input);  
	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_auto , Pitch_Motor[0]     , pitch_locationPID_input);  

	//û�м�⵽װ�װ�:
	if(top_Pitch == 32760)
	{	
	gimbal_speed_pitch_set =  PID_Calc(&pitchmotor_remote       , Pitch_Motor[0]     ,  Pitch_motorloco_init);   

	Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto     , gimbal_speed_yaw   ,  gimbal_speed_yaw_set),  
														PID_Calc(&pitchmotor_speed_remote , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
														PID_Calc(&triggermotor            , Trigger_Motor[1]   ,  Trigger_Motor_Set), 
														0); 
	}
	//�м�⵽װ�װ�:
	else
	{
	Send_motorvalue_CAN1_3(  -PID_Calc(&yawmotor_speed_auto   , gimbal_speed_yaw   , gimbal_speed_yaw_set), 
														PID_Calc(&pitchmotor_speed_auto , gimbal_speed_pitch , -gimbal_speed_pitch_set), 
														PID_Calc(&triggermotor          , Trigger_Motor[1]   , Trigger_Motor_Set), //
														0); 
	}
}

