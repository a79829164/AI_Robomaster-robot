#include "shoot_task.h"
#include "remote_task.h"
#include "pid.h"


extern PidTypeDef triggermotor;
u8 Set_Shoot_status;
int defend_stop_trigger_flag = 0 ,fire_the_hole_flag = 0,  defend_stop_trigger_cnt1 = 0 , defend_stop_trigger_cnt2 = 0;

extern uint16_t Bullet_freq;
uint16_t Bullet_freq_tar = 1000 , Bullet_freq_Kp = 20;

int shoot_flag = 0;
extern int fixed_bullet_speedset   ;
void shoot_task(void)
{
  KEY_STATE = GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_9) ;
 
	if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 0)//开启设置
	{
		TIM2->CCR4 = shoot_speed_value;     
		TIM4->CCR4 = shoot_speed_value;	 
		shoot_status = 1;
		Trigger_Motor_Set = 0;
	}
	else if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 1)//关闭摩擦轮
	{
		TIM2->CCR4 = 1000;      
		TIM4->CCR4 = 1000;	
		shoot_status = 0;
		Trigger_Motor_Set = 0;
	}

	if(RC_CtrlData.rc.s1 == 3 && last_rc_s1 == 2 && shoot_status == 1) 
	{
//		shoot_flag = 0;
		Trigger_Motor_Set = 0;
	}
  if(RC_CtrlData.rc.s1 == 2 && last_rc_s1 == 3 && shoot_status == 1) 
	{
		Trigger_Motor_Set = fixed_bullet_speedset;
//		shoot_flag = 1;
	}
	
	
	
//	//防反转：
//	//堵转判断：	//堵转计时：
//	if(shoot_flag == 0)
//	{
////		Trigger_Motor_Set = 0;
//		defend_stop_trigger_cnt1 = 0;
//		defend_stop_trigger_cnt2 = 0;
//	}
//	if(shoot_flag == 1 && Trigger_Motor[1] > 500)
//	{
//		defend_stop_trigger_cnt1 = 0;
//	}
//	if(defend_stop_trigger_flag == 0 && shoot_flag == 1 && Trigger_Motor[1] <= 500)
//	{
//		defend_stop_trigger_cnt1++;
//		if(defend_stop_trigger_cnt1 >= 500)
//		{
//			defend_stop_trigger_flag = 1;
//			defend_stop_trigger_cnt1 = 0;
//		}
//	}
//  //反转计时：
//  if(defend_stop_trigger_flag == 1)
//	{
//		Trigger_Motor_Set = -8000;
//		defend_stop_trigger_cnt2++;
//		if(defend_stop_trigger_cnt2 >= 500)
//		{
//			Trigger_Motor_Set = fixed_bullet_speedset;
//			defend_stop_trigger_flag = 0;
//			defend_stop_trigger_cnt2 = 0;
//		}
//	}

	
	
//  if(RC_CtrlData.rc.s1 == 3 && Set_Shoot_status == 1)//关闭摩擦轮
//	{
//		shoot_flag = 0;
//		Trigger_Motor_Set = 0;
//	}
//  if(RC_CtrlData.rc.s1 == 2 && Set_Shoot_status == 1)//关闭摩擦轮
//	{
//		shoot_flag = 1;
//	}
//		
//	//射击部分：
//	//////////////////////////////////////////////////////////////////
//	//启动射击命令：
//	if(shoot_start_flag == 1 && shoot_flag == 1 && shoot_status == 1)
//	{
//		Trigger_Motor_Set     = 10000;
//		if(KEY_STATE == 1 && LAST_KEY_STATE == 0)
//		{
//			shoot_bullet_cnt++;
//			triggermotor.out_ki = 0;
//			shoot_start_flag    = 0; 
//		}
//	}	
//	//开枪间隔计时：
//	if(shoot_start_flag == 0 && shoot_flag == 1 && shoot_status == 1)
//	{
//		shoot_frequence_cnt++;
//		Trigger_Motor_Set     = -1000;
//		if(shoot_frequence_cnt >= Bullet_freq_tar)//一秒1000 / 200 = 5发
//		{
//			shoot_frequence_cnt = 0;
//			shoot_start_flag    = 1;
//		}
//	}
//	//否则清零：
//	if(shoot_flag == 0)
//	{
//		Trigger_Motor_Set = 0;
//	}
//	
//	
	if(KEY_STATE == 1 && LAST_KEY_STATE == 0)
	{
		shoot_bullet_cnt++;
		triggermotor.out_ki = 0;
		shoot_start_flag    = 0; 
	}
		
  last_rc_s1 = RC_CtrlData.rc.s1;
	LAST_KEY_STATE = KEY_STATE;
}





	//子弹上膛：	
//	if(shoot_start_flag == 0 && Set_Shoot_status == 1)
//	{
//		if(KEY_STATE == 1 && shoot_bullet_cnt == 0)
//		{
//			Trigger_Motor_Set = -1000;
//		}
//		else if(KEY_STATE == 1 && shoot_bullet_cnt > 0)
//		{
//			Trigger_Motor_Set = -1000;
//		}
//		else if(KEY_STATE == 0)
//		{
//			Trigger_Motor_Set = 0;
//		}
//	}



	
//	if(RC_CtrlData.rc.s1 == 2 && last_rc_s1 == 3 && Set_Shoot_status == 1) //发射子弹
//	{
//		Trigger_Motor_Set = 4000;
//		Bullet_freq_tar = 8;
//		fire_the_hole_flag = 1;
//	}
//	if(RC_CtrlData.rc.s1 == 3) //发射子弹
//	{
//		Trigger_Motor_Set = 0;
//		fire_the_hole_flag = 0;
//	}
////	
////	//子弹计数：
////	if(fire_the_hole_flag == 1)
////	{
////		if(KEY_STATE == 1 && LAST_KEY_STATE == 0)
////		{
////			shoot_bullet_cnt++;
////		}
////	}
//		
////	//限射频：
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
//	//防反转：
//	//堵转判断：	//堵转计时：
//	if(shoot_flag == 0)
//	{
//		defend_stop_trigger_cnt1 = 0;
//		defend_stop_trigger_cnt2 = 0;
//	}
//	if(shoot_flag == 1 && Trigger_Motor[1] > 500)
//	{
//		defend_stop_trigger_cnt1 = 0;
//	}
//	if(defend_stop_trigger_flag == 0 && shoot_flag == 1 && Trigger_Motor[1] <= 500)
//	{
//		defend_stop_trigger_cnt1++;
//		if(defend_stop_trigger_cnt1 >= 1000)
//		{
//			defend_stop_trigger_flag = 1;
//			defend_stop_trigger_cnt1 = 0;
//		}
//	}
//  //反转计时：
//  if(defend_stop_trigger_flag == 1)
//	{
//		Trigger_Motor_Set = -8000;
//		defend_stop_trigger_cnt2++;
//		if(defend_stop_trigger_cnt2 >= 500)
//		{
//			Trigger_Motor_Set = 4500;
//			defend_stop_trigger_flag = 0;
//			defend_stop_trigger_cnt2 = 0;
//		}
//	}


 