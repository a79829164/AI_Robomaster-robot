#include "stm32f4xx.h" 
#include "value.h"

#define rx_buf_length 32

typedef struct
{
	int8_t  Top_Chassis_Mode;//�ϲ�ģʽָ��
	int8_t  Top_Gimbal_Mode;//�ϲ�ģʽָ��
	int16_t Top_velocity_x;//�ϲ㷽��ָ��
	int16_t Top_velocity_y ;//�ϲ��ٶ�ָ��
	int16_t Top_velocity_yaw ;//�ϲ��ٶ�ָ��
	int16_t Top_Yaw;   //�ϲ�Yaw����תָ��
	int16_t Top_Pitch; //�ϲ�Pitch��Ƕ�ָ��
	int16_t Top_distance;    

}Top_Data_t;


extern Top_Data_t Top_Data;
void usart2_config(float bound);
void SendMessage_to_Ros();



