#include "gimbal_task.h"
#include "pid.h"
#include "remote_task.h"


 

void gimbal_task(void)
{
	//ң��ģʽ
	if(RC_CtrlData.rc.s2 == 1)
	{
		gimbal_speed_yaw_set  =  PID_Calc(&yawmotor_angle , ZGyroModuleAngle , -yaw_angle_set); // -Ϊ˳ʱ��
		gimbal_speed_pitch_set = PID_Calc(&pitchmotor_angle , Pitch_Motor[0] , pitch_angle_set + Pitch_motorloco_init); // -Ϊ˳ʱ��  
		
		Send_motorvalue_CAN1_3(-PID_Calc(&yawmotor_speed , gimbal_speed_yaw , gimbal_speed_yaw_set),// -Ϊ˳ʱ��
														PID_Calc(&pitchmotor_speed , gimbal_speed_pitch , -gimbal_speed_pitch_set),// -Ϊ����̧
														PID_Calc(&triggermotor , Trigger_Motor[1] , Trigger_Motor_Set),//
														0); 
	}
 
  //�Զ�ģʽ
	if(RC_CtrlData.rc.s2 == 3)
	{
		gimbal_speed_yaw_set  =  PID_Calc(&yawmotor_position , top_dx , 0); // -Ϊ˳ʱ�� 
		gimbal_speed_pitch_set = PID_Calc(&pitchmotor_position , top_dy , 0); // -Ϊ˳ʱ��  
		
		Send_motorvalue_CAN1_3(-PID_Calc(&yawmotor_speed , gimbal_speed_yaw , gimbal_speed_yaw_set),// -Ϊ˳ʱ��
														PID_Calc(&pitchmotor_speed , gimbal_speed_pitch , -gimbal_speed_pitch_set),// -Ϊ����̧
														PID_Calc(&triggermotor , Trigger_Motor[1] , Trigger_Motor_Set),//
														0); 
	}
	
	//stop
	if(RC_CtrlData.rc.s2 == 2)
	{
		gimbal_speed_yaw_set = 0;
		gimbal_speed_pitch_set = 0;
		
		Send_motorvalue_CAN1_3(-PID_Calc(&yawmotor_speed , gimbal_speed_yaw , gimbal_speed_yaw_set),// -Ϊ˳ʱ��
												PID_Calc(&pitchmotor_speed , gimbal_speed_pitch , -gimbal_speed_pitch_set),// -Ϊ����̧
												PID_Calc(&triggermotor , Trigger_Motor[1] , Trigger_Motor_Set),//
												0); 
	}
}



