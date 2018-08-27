#include "shoot_task.h"
#include "remote_task.h"
#include "pid.h"


extern PidTypeDef triggermotor;

int defend_stop_trigger_flag = 0 ,fire_the_hole_flag = 0,  defend_stop_trigger_cnt1 = 0 , defend_stop_trigger_cnt2 = 0;

extern uint16_t Bullet_freq;
uint16_t Bullet_freq_tar = 1000 , Bullet_freq_Kp = 20;

int shoot_flag;
void shoot_task(void)
{
  KEY_STATE = GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_9) ;
	
	if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 0)//��������
	{
		TIM2->CCR1 = TIM2_CCR1_value;     
		TIM4->CCR1 = TIM4_CCR1_value;	 
		shoot_status = 1;
		Trigger_Motor_Set = 0;
	}
	else if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 1)
	{
		TIM2->CCR1 = 50;      
		TIM4->CCR1 = 500;	
		shoot_status = 0;
		Trigger_Motor_Set  = 0;
	}
//	if(RC_CtrlData.rc.s1 == 3 && last_rc_s1 == 2 && shoot_status == 1)
//	{
////		shoot_flag = 0;
//		Trigger_Motor_Set = 0;
//	}
//  if(RC_CtrlData.rc.s1 == 2 && last_rc_s1 == 3 && shoot_status == 1)
//	{
////		shoot_flag = 1;
//		Trigger_Motor_Set = 2000;
//	}
//	
//	////////////////////////////////////////////////////////////////////
//	//����������
//	if(shoot_start_flag == 1 && shoot_flag == 1 && shoot_status == 1)
//	{
//		Trigger_Motor_Set = 20000;//20000
//		if(KEY_STATE == 1 && LAST_KEY_STATE == 0)
//		{
//			shoot_bullet_cnt++;
//			triggermotor.out_ki = 0;
//			shoot_start_flag  = 0; 
//		}
//	}
//	//��ǹ�����ʱ��
//	if(shoot_start_flag == 0 && shoot_flag == 1 && shoot_status == 1)
//	{
//		shoot_frequence_cnt++;
//		Trigger_Motor_Set = -1000;
//		if(shoot_frequence_cnt >= Bullet_freq_tar)//һ��1000 / 200 =5��
//		{
//			shoot_frequence_cnt = 0;
//			shoot_start_flag = 1;
//		}
//	}
//	//����ֱ�ӹرղ�����
//	if(shoot_flag == 0)
//	{
//		Trigger_Motor_Set = 0;
//	}
//	
	
	
	
//	///////////////////////////////////////////////////////////////////////////////////
//	if(RC_CtrlData.rc.s1 == 2 && last_rc_s1 == 3 && shoot_status == 1) //�����ӵ�
//	{
//		Trigger_Motor_Set = 10000;
//		Bullet_freq_tar = 8;
//		fire_the_hole_flag = 1;
//	}
//	if(RC_CtrlData.rc.s1 == 3) //�����ӵ�
//	{
//		Trigger_Motor_Set = 0;
//		fire_the_hole_flag = 0;
//	}
//	
//	//�ӵ�������
//	if(fire_the_hole_flag == 1)
//	{
//		if(KEY_STATE == 1 && LAST_KEY_STATE == 0)
//		{
//			shoot_bullet_cnt++;
//		}
//	}
	
	
	
	
	
	
//		
////	//����Ƶ��
////	if(fire_the_hole_flag == 1 && defend_stop_trigger_flag == 0)
////	{
////		Trigger_Motor_Set = 6000;
//////		if(Bullet_freq_tar >= Bullet_freq)
//////		{
//////			Trigger_Motor_Set = Trigger_Motor_Set + Bullet_freq_Kp;
//////		}
//////		else
//////		{
//////		  Trigger_Motor_Set = Trigger_Motor_Set - Bullet_freq_Kp;
//////		}
////	}
////	
//	//����ת��
//	//��ת�жϣ�	//��ת��ʱ��
//	if(fire_the_hole_flag == 0)
//	{
//		defend_stop_trigger_cnt1 = 0;
//		defend_stop_trigger_cnt2 = 0;
//	}
//	if(fire_the_hole_flag == 1 && Trigger_Motor[1] > 500)
//	{
//		defend_stop_trigger_cnt1 = 0;
//	}
//	if(defend_stop_trigger_flag == 0 && fire_the_hole_flag == 1 && Trigger_Motor[1] <= 500)
//	{
//		defend_stop_trigger_cnt1++;
//		if(defend_stop_trigger_cnt1 >= 500)
//		{
//			defend_stop_trigger_flag = 1;
//			defend_stop_trigger_cnt1 = 0;
//		}
//	}
//  //��ת��ʱ��
//  if(defend_stop_trigger_flag == 1)
//	{
//		Trigger_Motor_Set = -8000;
//		defend_stop_trigger_cnt2++;
//		if(defend_stop_trigger_cnt2 >= 500)
//		{
//			Trigger_Motor_Set = 3000;
//			defend_stop_trigger_flag = 0;
//			defend_stop_trigger_cnt2 = 0;
//		}
//	}


	
  last_rc_s1 = RC_CtrlData.rc.s1;
	LAST_KEY_STATE = KEY_STATE;
}



 
//	
//	if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 0)//��������
//	{
//		TIM2->CCR1 = TIM2_CCR1_value;     
//		TIM4->CCR1 = TIM4_CCR1_value;	 
//		shoot_status = 1;
//	}
//	else if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 1)//�ر�Ħ����
//	{
//		TIM2->CCR1 = 50;      
//		TIM4->CCR1 = 500;	
//		shoot_status = 0;
//	}
//	if(RC_CtrlData.rc.s1 == 2 && last_rc_s1 == 3 && shoot_status == 1) //�����ӵ�
//	{
//		Trigger_Motor_Set = 10000;
//	}
//	if(RC_CtrlData.rc.s1 == 3) // 
//	{
//		Trigger_Motor_Set = 0;
//	}
	