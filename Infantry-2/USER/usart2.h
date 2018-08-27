#include "stm32f4xx.h" 
#include "value.h"

#define rx_buf_length 32

typedef struct
{
	int8_t  Top_Chassis_Mode;//上层模式指令
	int8_t  Top_Gimbal_Mode;//上层模式指令
	int16_t Top_velocity_x;//上层方向指令
	int16_t Top_velocity_y ;//上层速度指令
	int16_t Top_velocity_yaw ;//上层速度指令
	int16_t Top_Yaw;   //上层Yaw轴旋转指令
	int16_t Top_Pitch; //上层Pitch轴角度指令
	int16_t Top_distance;    

}Top_Data_t;


extern Top_Data_t Top_Data;
void usart2_config(float bound);
void SendMessage_to_Ros();



