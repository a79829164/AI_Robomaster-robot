#include "remote_task.h"

RC_CtrlData_t RC_CtrlData;
int rc_ch0_value , rc_ch1_value , rc_ch2_value , rc_ch3_value;

void RemoteDataProcess(uint8_t *pData)
{
	if(pData == NULL)
	{
		return;
	}
	
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;

	RC_CtrlData.rc.s1  = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2  = ((pData[5] >> 4) & 0x0003)  ;
		
	RC_CtrlData.mouse.x  = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y  = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z  = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];	
	
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8); 
	
	switch(RC_CtrlData.rc.s2)//控制端
	{
		case REMOTE_INPUT://手动遥控
		{
			remote_control();
		}break;
		
		case AUTOMATIC_INPUT://自动模式
		{
			automatic_control();
		}break;
		
		case STOP:           //全车静止
		{
			rc_ch0_value = 0;
			rc_ch1_value = 0;
	    chassis_Motor_M1_set = 0; 
	    chassis_Motor_M2_set = 0; 
	    chassis_Motor_M3_set = 0; 
	    chassis_Motor_M4_set = 0; 			
			yaw_angle_set = -ZGyroModuleAngle;
		}break;
	}
}


float para_ch0_1 = 0.3 , para_ch2_3 = 0.3 , yaw_angle_para = 0.003 , pitch_angle_para = 0.05;
extern float gimbal_chassis_err ; 
void remote_control(void)
{
	rc_ch0_value = (RC_CtrlData.rc.ch0 - 1024 - 4) * para_ch0_1;
	rc_ch1_value = (RC_CtrlData.rc.ch1 - 1024)     * para_ch0_1;
	rc_ch2_value = (RC_CtrlData.rc.ch2 - 1024)     * para_ch2_3;        //* para_ch2_3;
	rc_ch3_value = (RC_CtrlData.rc.ch3 - 1024)     * para_ch2_3;

//	
//	///云台运动期望
//	yaw_angle_set += rc_ch2_value * yaw_angle_para;
//	pitch_angle_set += rc_ch3_value * pitch_angle_para;
//	if(pitch_angle_set >= 200)
//	{
//		pitch_angle_set = 200;
//	}
//	else if(pitch_angle_set <= -500)
//	{
//		pitch_angle_set = -500;
//	}
	
}

//////////////////////////////////////////////////////////
 

void automatic_control(void)
{
	switch(automatic_control_mode)
	{
    case ranger_mode:
		{
			ranger_movement(); //游荡模式
		}break;
		
		case attack_mode:
		{
			attack_movement(); //攻击模式
		}break;
	}
}

void ranger_movement(void) //游荡模式
{
  switch(communication_temp)
	{
		case 'p':  //停止
		{
			chassis_Motor_M1_set = 0;
			chassis_Motor_M2_set = 0;
			chassis_Motor_M3_set = 0;
			chassis_Motor_M4_set = 0;
		}break;
		
		case 'w':  //前进
		{
			chassis_Motor_M1_set = -660 * 0.4;
			chassis_Motor_M2_set =  660 * 0.4;
			chassis_Motor_M3_set =  660 * 0.4;
			chassis_Motor_M4_set = -660 * 0.4;
		}break;
		
		case 's':  //后退
		{
			chassis_Motor_M1_set =  660 * 0.4;
			chassis_Motor_M2_set = -660 * 0.4;
			chassis_Motor_M3_set = -660 * 0.4;
			chassis_Motor_M4_set =  660 * 0.4;
		}break;
		
		case 'a':  //左移 97
		{
			chassis_Motor_M1_set = -660 * 0.4;
			chassis_Motor_M2_set = -660 * 0.4;
			chassis_Motor_M3_set =  660 * 0.4;
			chassis_Motor_M4_set =  660 * 0.4;
		}break;
		
		case 'd':  //左移 100
		{
			chassis_Motor_M1_set =  660 * 0.4;
			chassis_Motor_M2_set =  660 * 0.4;
			chassis_Motor_M3_set = -660 * 0.4;
			chassis_Motor_M4_set = -660 * 0.4;
		}break;		
		
  	case 'q':  //左转 
		{
      yaw_angle_set +=       -360 * 0.003;
		}break;		
		
		case 'e':  //右转
		{
      yaw_angle_set +=        360 * 0.003;
		}break;	
		
	}
		
}


void attack_movement(void)
{
	
}
	