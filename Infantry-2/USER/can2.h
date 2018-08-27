
#include "stm32f4xx.h" 

#include "value.h"



typedef struct
{
	int16_t x_uwb;// 
	int16_t y_uwb;// 
	float yaw_angle_uwb ;// 
	float init_yaw_angle_uwb ;// 
	int16_t distance_uwb[4];   // 
	int16_t error_type_signal_uwb; // 
	int16_t reserve_uwb;    // 
}Uwb_data_t;

//UWB接收数据：
extern Uwb_data_t Uwb_data;


void Can2_Configuration(void);

void Send_RMGyro_Reset(void);