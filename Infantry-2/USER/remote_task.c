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
		
		case STOP: //全车静止 
		{
			rc_ch0_value = 0;
			rc_ch1_value = 0;
	    chassis_Motor_M1_set = 0; 
	    chassis_Motor_M2_set = 0; 
	    chassis_Motor_M3_set = 0; 
	    chassis_Motor_M4_set = 0; 			
//			yaw_angle_set = -ZGyroModuleAngle;		
		}break;
		
		case AUTOMATIC_INPUT:  //自动模式
		{
 
		}break;
	}
}


float para_ch0_1 = 0.2 , para_ch2_3 = 0.81 , yaw_angle_para = 0.075 , pitch_angle_para = 0.05;
extern float gimbal_chassis_err ; 
void remote_control(void)
{
	rc_ch0_value = (RC_CtrlData.rc.ch0 - 1024) * para_ch0_1;
	rc_ch1_value = (RC_CtrlData.rc.ch1 - 1024) * para_ch0_1;
	rc_ch2_value = (RC_CtrlData.rc.ch2 - 1024) * para_ch2_3;
	rc_ch3_value = (RC_CtrlData.rc.ch3 - 1024) * para_ch2_3;

	///云台运动期望
	yaw_angle_set += rc_ch2_value * yaw_angle_para;
	if(yaw_angle_set >= 1700)
	{
		yaw_angle_set = 1700;
	}
	if(yaw_angle_set <= -1700)
	{
		yaw_angle_set = -1700;
	}
	
	pitch_angle_set += rc_ch3_value * pitch_angle_para;
	if(pitch_angle_set >= 200)
	{
		pitch_angle_set = 200;
	}
	else if(pitch_angle_set <= -500)
	{
		pitch_angle_set = -500;
	}
	
}

//////////////////////////////////////////////////////////
 
