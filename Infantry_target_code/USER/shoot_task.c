#include "shoot_task.h"
#include "remote_task.h"

u8 last_rc_s1 = 0;
u8 shoot_status = 0;
void shoot_task(void)
{
	if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 0)//¿ªÆôÄ¦²ÁÂÖ
	{
		shoot_status = 1;
    TIM2->CCR1 = 64;   //   64
    TIM4->CCR1 = 770;	 //   770
	}
	else if(RC_CtrlData.rc.s1 == 1 && last_rc_s1 == 3 && shoot_status == 1)//¹Ø±ÕÄ¦²ÁÂÖ
	{
		shoot_status = 0;
    TIM2->CCR1 = 50;   //   64
    TIM4->CCR1 = 500;	 //   770
	}

	////////////////////////
	
	if(RC_CtrlData.rc.s1 == 2 && shoot_status == 1) //·¢Éä×Óµ¯
	{
		Trigger_Motor_Set = 2000;
	}
	else
	{
		Trigger_Motor_Set = 0;
	}
	
  last_rc_s1 = RC_CtrlData.rc.s1;
}







