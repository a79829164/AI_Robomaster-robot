#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "laser.h"
#include "usart1.h" 
#include "usart2.h" 
#include "tim3.h"
#include "can1.h"
#include "duty.h"
#include "pid.h"
#include "tim1_pwm.h"
#include "led.h"
#include "buzzer.h"
#include "can2.h"
#include "judgeuart.h" 

 //导航数据初始化：
void Map_data_init(void);

int main(void)
{ 
 	SystemInit();
	buzzer_config();
	Shoot_limit_Init();
	guangdian_switch_Init();
	led_config();
  laser_config();
	Can1_Configuration();
  Can2_Configuration();
	delay_ms(1000);
	usart1_config(100000); 
	usart2_config(115200);
	usart6_config();
	SPI5_Init();
	mpu_device_init();
  PID_InitALL();
	Map_data_init();
	Tim5_config();
	Tim3_config();
	tim1_pwm_config();
	Send_RMGyro_Reset();
	while(ZGyroModuleAngle != 0)
	{
		break;
	}
 
	
	while(1)
	{
		loop();
	}
}
 

