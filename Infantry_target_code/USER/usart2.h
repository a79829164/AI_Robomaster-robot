#include "stm32f4xx.h" 
#include "value.h"

#define rx_buf_length 32

typedef struct
{
	int16_t Top_Mode;//�ϲ�ģʽָ��
	int16_t Top_Direction;//�ϲ㷽��ָ��
	int16_t Top_Speed ;//�ϲ��ٶ�ָ��
	int16_t Top_Yaw;   //�ϲ�Yaw����תָ��
	int16_t Top_Pitch; //�ϲ�Pitch��Ƕ�ָ��
	int16_t Top_dx;    //�ϲ�ͼ������X���ֵ
	int16_t Top_dy;    //�ϲ�ͼ������Y���ֵ

}Top_Data_t;

void usart2_config(float bound);




