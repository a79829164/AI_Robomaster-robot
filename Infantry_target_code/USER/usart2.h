#include "stm32f4xx.h" 
#include "value.h"

#define rx_buf_length 32

typedef struct
{
	int16_t Top_Mode;//上层模式指令
	int16_t Top_Direction;//上层方向指令
	int16_t Top_Speed ;//上层速度指令
	int16_t Top_Yaw;   //上层Yaw轴旋转指令
	int16_t Top_Pitch; //上层Pitch轴角度指令
	int16_t Top_dx;    //上层图像数据X轴差值
	int16_t Top_dy;    //上层图像数据Y轴差值

}Top_Data_t;

void usart2_config(float bound);




