#include "gimbal_task.h"
#include "pid.h"
#include "remote_task.h"


 

void gimbal_task(void)
{
	//遥控模式
	if(RC_CtrlData.rc.s2 == 1)
	{
		gimbal_speed_yaw_set  =  PID_Calc(&yawmotor_angle , ZGyroModuleAngle , -yaw_angle_set); // -为顺时针
		gimbal_speed_pitch_set = PID_Calc(&pitchmotor_angle , Pitch_Motor[0] , pitch_angle_set + Pitch_motorloco_init); // -为顺时针  
		
		Send_motorvalue_CAN1_3(-PID_Calc(&yawmotor_speed , gimbal_speed_yaw , gimbal_speed_yaw_set),// -为顺时针
														PID_Calc(&pitchmotor_speed , gimbal_speed_pitch , -gimbal_speed_pitch_set),// -为向上抬
														PID_Calc(&triggermotor , Trigger_Motor[1] , Trigger_Motor_Set),//
														0); 
	}
 
  //自动模式
	if(RC_CtrlData.rc.s2 == 3)
	{
		gimbal_speed_yaw_set  =  PID_Calc(&yawmotor_position , top_dx , 0); // -为顺时针 
		gimbal_speed_pitch_set = PID_Calc(&pitchmotor_position , top_dy , 0); // -为顺时针  
		
		Send_motorvalue_CAN1_3(-PID_Calc(&yawmotor_speed , gimbal_speed_yaw , gimbal_speed_yaw_set),// -为顺时针
														PID_Calc(&pitchmotor_speed , gimbal_speed_pitch , -gimbal_speed_pitch_set),// -为向上抬
														PID_Calc(&triggermotor , Trigger_Motor[1] , Trigger_Motor_Set),//
														0); 
	}
	
	//stop
	if(RC_CtrlData.rc.s2 == 2)
	{
		gimbal_speed_yaw_set = 0;
		gimbal_speed_pitch_set = 0;
		
		Send_motorvalue_CAN1_3(-PID_Calc(&yawmotor_speed , gimbal_speed_yaw , gimbal_speed_yaw_set),// -为顺时针
												PID_Calc(&pitchmotor_speed , gimbal_speed_pitch , -gimbal_speed_pitch_set),// -为向上抬
												PID_Calc(&triggermotor , Trigger_Motor[1] , Trigger_Motor_Set),//
												0); 
	}
}



