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


int main(void)
{ 
	SystemInit();
	buzzer_config();
	led_config();
  laser_config();
	tim1_pwm_config();
	usart1_config(100000);
	usart2_config(115200);
	SPI5_Init();
	mpu_device_init();
  PID_InitALL();
	Tim3_config();
	Can1_Configuration();
//  Can2_Configuration();
//	Send_RMGyro_Reset();
//	while(ZGyroModuleAngle != 0)
//	{
//		break;
//	}
//	
	while(1)
	{
    loop();
	}
}
 

