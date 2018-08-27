#include "duty.h"
#include "remote_task.h"
#include "usart2.h" 
#include "can2.h"
#include "navigation_task.h"

//int uwb_Selfcheck_flag = 0;//1为刚上电  0为没断电
void loop(void)
{

	SendMessage_to_Ros();
//  automatic_control();
}


void Single_point_navi_Test(void);
void chassis_task(void);
void shoot_task(void);
void gimbal_task(void);

float uwb_selfcheck_cnt;
//开枪 ：
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM2,TIM_FLAG_Update);


 
	}
}

//开机uwb自检时间：
int uwb_Selfcheck_flag = 0; //1为刚上电  0为没断电
//IMU + Gimbal运行
int dwqdwqdw = 0 , k_flag111 = 0 , M1 , M2 , M3 , M4;
extern int usart1_com_cnt , usart1_com_ornot_cnt, usart1_com_ornot_cnt_last;
void TIM3_IRQHandler(void) //1000hz
{
	if(TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM3,TIM_FLAG_Update);

		//比赛倒计时： 
		if(RC_CtrlData.rc.s2 == AUTOMATIC_INPUT && game_start_flag == 0)
		{
		  game_start_cnt++;
			if(game_start_cnt >= 5000)
			{
				game_start_cnt = 0;
				game_start_flag = 1;
			}
		}
		
		
		//接收遥控器信号：
		usart1_com_cnt++;
		if(usart1_com_cnt >= 100)
		{
			usart1_com_cnt = 0;
			if(f(usart1_com_ornot_cnt_last - usart1_com_ornot_cnt) <= 1)
			{
      	NVIC_SystemReset();				
			}
			usart1_com_ornot_cnt_last = usart1_com_ornot_cnt;
		}
		
		/////imu + gimbal task
		imu_task();
		gimbal_task();

	}
}

//导航运行：
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM5,TIM_FLAG_Update);
   
	  shoot_task();
		
		
//		
//		if(uwb_Selfcheck_flag == 2)//UWB自检时间到，才可以控制小车
//		{
			chassis_task();		
//		}
 
	}
}
 
//与ros通信：
int TIM4_send_cnt = 0;
void  SendMessage_to_Ros();
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
		
	  //uwb自检计时：
		if(uwb_Selfcheck_flag == 1)  //刚上电
		{
			uwb_selfcheck_cnt++;
			if(uwb_selfcheck_cnt >= 22500)
			{
				uwb_Selfcheck_flag = 2;
				Uwb_data.init_yaw_angle_uwb = Uwb_data.yaw_angle_uwb;
				Map_data.point_init.x       = Uwb_data.x_uwb;
				Map_data.point_init.y       = Uwb_data.y_uwb;
			}		
		}
		if(uwb_Selfcheck_flag == 0) //没断电
		{
			uwb_selfcheck_cnt++;
			if(uwb_selfcheck_cnt >= 250)
			{
				uwb_Selfcheck_flag = 2;
				Uwb_data.init_yaw_angle_uwb = Uwb_data.yaw_angle_uwb;
				Map_data.point_init.x       = Uwb_data.x_uwb;
				Map_data.point_init.y       = Uwb_data.y_uwb;
			}	
		}
	}
}

//串口接收中断服务函数
int BSP_USART1_DMA_RX_BUF_LEN = 30;
uint8_t _USART1_DMA_RX_BUF[2][30];
int usart1_com_cnt , usart1_com_ornot_cnt = 1 , usart1_com_ornot_cnt_last , usart1_com_ornot_flag;
void USART1_IRQHandler(void)
{
	usart1_com_ornot_cnt++;
	if(usart1_com_ornot_cnt >= 500)
	{
		usart1_com_ornot_cnt = 1;
	}
	
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART1->SR;
		(void)USART1->DR;

		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream2, ENABLE);
      if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				RemoteDataProcess(_USART1_DMA_RX_BUF[0]);
			}
		}
		//Target is Memory1
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream2, ENABLE);
      if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				RemoteDataProcess(_USART1_DMA_RX_BUF[1]);
			}
		}
	}       
}


////串口接收中断服务函数
extern u8 automatic_control_mode;
char rxbuf[rx_buf_length];// 接收数据的缓冲区

//前方激光雷达测距距离：
extern s16 front_distance_from_lidar;

//串口接收中断服务函数
void USART2_IRQHandler(void)
{
	u8 ccnt;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART2->SR;
		(void)USART2->DR;
		
    DMA_Cmd(DMA1_Stream5,DISABLE);  
   
    DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);  
 
    for(int i = 0; i < rx_buf_length / 2; i++)
		{
		  if(rxbuf[i] == 0x7F)    //接收到包头
			{  
				ccnt = i;
        break;
			}
		}
 
	  if(rxbuf[ccnt + 15] == 0x7E)    //接收到包尾                  
		{
			Top_Data.Top_Chassis_Mode =  rxbuf[ccnt + 1];
     	Top_Data.Top_Gimbal_Mode  =  rxbuf[ccnt + 2];
			Top_Data.Top_velocity_x   = (rxbuf[ccnt + 4]  << 8) | rxbuf[ccnt + 3];
			Top_Data.Top_velocity_y   = (rxbuf[ccnt + 6]  << 8) | rxbuf[ccnt + 5];
			Top_Data.Top_velocity_yaw = (rxbuf[ccnt + 8]  << 8) | rxbuf[ccnt + 7];
			Top_Data.Top_Yaw          = (rxbuf[ccnt + 10] << 8) | rxbuf[ccnt + 9];
			Top_Data.Top_Pitch        = (rxbuf[ccnt + 12] << 8) | rxbuf[ccnt + 11];
			Top_Data.Top_distance     = (rxbuf[ccnt + 14] << 8) | rxbuf[ccnt + 13];        
			
			if(Top_Data.Top_Chassis_Mode == 3)
			{
        front_distance_from_lidar = Top_Data.Top_distance;
			}
			
			top_Yaw                   =  Top_Data.Top_Yaw;
			top_Pitch                 =  Top_Data.Top_Pitch;
			top_distance              =  Top_Data.Top_distance / 100.0;
 
		}
		
    DMA_SetCurrDataCounter(DMA1_Stream5,rx_buf_length);  
    DMA_Cmd(DMA1_Stream5,ENABLE);  
	}
}


int startcan1_flag_cnt = 0;
int chassismotor_cnt1 , chassismotor_cnt2 , chassismotor_cnt3 , chassismotor_cnt4;
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);		
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);	
		
		if(startcan1_flag_cnt < 1000)
		{
			startcan1_flag_cnt++;
		}
		if(startcan1_flag_cnt >= 1000)
		{
			startcan1_flag_cnt = 1000;
		}
			
		switch(rx_message.StdId)
		{
			case CAN_3510_M1_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M1[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M1ofset = chassis_Motor_M1[0];
				}
				else
				{
					chassis_Motor_M1_last = chassis_Motor_M1[0];
					chassis_Motor_M1[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M1[0] - chassis_Motor_M1_last > 4096)
					{
						chassis_Motor_M1_cnt--;
						chassis_Motor_M1[3] = chassis_Motor_M1[0] - chassis_Motor_M1_last - 8192;
					}
					else if(chassis_Motor_M1[0] - chassis_Motor_M1_last < -4096)
					{
						chassis_Motor_M1_cnt++;
						chassis_Motor_M1[3] = chassis_Motor_M1[0] - chassis_Motor_M1_last + 8192;
					}
					else 
					{
						chassis_Motor_M1[3] = chassis_Motor_M1[0] - chassis_Motor_M1_last ;
					}
					chassis_Motor1_loco = chassis_Motor_M1_cnt * 8192 + chassis_Motor_M1[0] - chassis_Motor_M1ofset;					
				}
			  int32_t temp_sum_1 = 0;
				chassis_Motor_M1_buf[chassismotor_cnt1++] = chassis_Motor_M1[3];
        if(chassismotor_cnt1 >= FILTER_BUF) 
				{
					chassismotor_cnt1 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_1 += chassis_Motor_M1_buf[i];
				}
				chassis_Motor_M1[1] = (int16_t)(temp_sum_1 / FILTER_BUF * 7.324f);
			break;
				
			case CAN_3510_M2_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M2[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M2ofset = chassis_Motor_M2[0];
				}
				else
				{
					chassis_Motor_M2_last = chassis_Motor_M2[0];
					chassis_Motor_M2[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M2[0] - chassis_Motor_M2_last > 4096)
					{
						chassis_Motor_M2_cnt--;
						chassis_Motor_M2[3] = chassis_Motor_M2[0] - chassis_Motor_M2_last - 8192;
					}
					else if(chassis_Motor_M2[0] - chassis_Motor_M2_last < -4096)
					{
						chassis_Motor_M2_cnt++;
						chassis_Motor_M2[3] = chassis_Motor_M2[0] - chassis_Motor_M2_last + 8192;
					}
					else 
					{
						chassis_Motor_M2[3] = chassis_Motor_M2[0] - chassis_Motor_M2_last ;
					}
					chassis_Motor2_loco = chassis_Motor_M2_cnt * 8192 + chassis_Motor_M2[0] - chassis_Motor_M2ofset;					
				}
			  int32_t temp_sum_2 = 0;
				chassis_Motor_M2_buf[chassismotor_cnt2++] = chassis_Motor_M2[3];
        if(chassismotor_cnt2 >= FILTER_BUF) 
				{
					chassismotor_cnt2 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_2 += chassis_Motor_M2_buf[i];
				}
				chassis_Motor_M2[1] = (int16_t)(temp_sum_2 / FILTER_BUF * 7.324f);
				
			break;	

			case CAN_3510_M3_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M3[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M3ofset = chassis_Motor_M3[0];
				}
				else
				{
					chassis_Motor_M3_last = chassis_Motor_M3[0];
					chassis_Motor_M3[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M3[0] - chassis_Motor_M3_last > 4096)
					{
						chassis_Motor_M3_cnt--;
						chassis_Motor_M3[3] = chassis_Motor_M3[0] - chassis_Motor_M3_last - 8192;
					}
					else if(chassis_Motor_M3[0] - chassis_Motor_M3_last < -4096)
					{
						chassis_Motor_M3_cnt++;
						chassis_Motor_M3[3] = chassis_Motor_M3[0] - chassis_Motor_M3_last + 8192;
					}
					else 
					{
						chassis_Motor_M3[3] = chassis_Motor_M3[0] - chassis_Motor_M3_last ;
					}
					chassis_Motor3_loco = chassis_Motor_M3_cnt * 8192 + chassis_Motor_M3[0] - chassis_Motor_M3ofset;					
				}
			  int32_t temp_sum_3 = 0;
				chassis_Motor_M3_buf[chassismotor_cnt3++] = chassis_Motor_M3[3];
        if(chassismotor_cnt3 >= FILTER_BUF) 
				{
					chassismotor_cnt3 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_3 += chassis_Motor_M3_buf[i];
				}
				chassis_Motor_M3[1] = (int16_t)(temp_sum_3 / FILTER_BUF * 7.324f);
				
			break;					
				
			case CAN_3510_M4_ID:
        if(startcan1_flag_cnt < 1000)
				{
		  		chassis_Motor_M4[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					chassis_Motor_M4ofset = chassis_Motor_M4[0];
				}
				else
				{
					chassis_Motor_M4_last = chassis_Motor_M4[0];
					chassis_Motor_M4[0] = (uint16_t)(rx_message.Data[0] << 8 | rx_message.Data[1]);
					if(chassis_Motor_M4[0] - chassis_Motor_M4_last > 4096)
					{
						chassis_Motor_M4_cnt--;
						chassis_Motor_M4[3] = chassis_Motor_M4[0] - chassis_Motor_M4_last - 8192;
					}
					else if(chassis_Motor_M4[0] - chassis_Motor_M4_last < -4096)
					{
						chassis_Motor_M4_cnt++;
						chassis_Motor_M4[3] = chassis_Motor_M4[0] - chassis_Motor_M4_last + 8192;
					}
					else 
					{
						chassis_Motor_M4[3] = chassis_Motor_M4[0] - chassis_Motor_M4_last ;
					}
					chassis_Motor4_loco = chassis_Motor_M4_cnt * 8192 + chassis_Motor_M4[0] - chassis_Motor_M4ofset;					
				}
			  int32_t temp_sum_4 = 0;
				chassis_Motor_M4_buf[chassismotor_cnt4++] = chassis_Motor_M4[3];
        if(chassismotor_cnt4 >= FILTER_BUF) 
				{
					chassismotor_cnt4 = 0;
				}
				for (uint8_t i = 0; i < FILTER_BUF; i++)
				{
					temp_sum_4 += chassis_Motor_M4_buf[i];
				}
				chassis_Motor_M4[1] = (int16_t)(temp_sum_4 / FILTER_BUF * 7.324f);
		
			break;						
				
			case CAN_YAW_MOTOR_ID:
        if(startcan1_flag_cnt < 1000)
				{
				}
				else
				{		
					DecodeS16Data(&Yaw_Motor[0] , &rx_message.Data[0]);
				}
			break;			
				
			case CAN_PITCH_MOTOR_ID:
        if(startcan1_flag_cnt < 1000)
				{
				}
				else
				{
					DecodeS16Data(&Pitch_Motor[0] , &rx_message.Data[0]);
				}
			break;			
			
			case CAN_TRIGGER_MOTOR_ID:
        if(startcan1_flag_cnt < 1000)
				{
				}
				else
				{
					DecodeS16Data(&Trigger_Motor[0] , &rx_message.Data[0]);
					DecodeS16Data(&Trigger_Motor[1] , &rx_message.Data[2]);
					DecodeS16Data(&Trigger_Motor[2] , &rx_message.Data[4]);
					Trigger_Motor[3] = rx_message.Data[6];
				}
			break;			

		}
	}
}
		

//UWB接收数据：

u8 rx_message_DLC_flag , rx_message_DLC_8_1 , rx_message_DLC_8_2 , sendtoROS_uwb_flag1;
int asddwfw;
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);		
		CAN_Receive(CAN2, CAN_FIFO0, &rx_message);	
	}
	
	switch(rx_message.StdId)
	{			
//		case CAN_GIMBAL_ZGYRO_ID://读单轴陀螺仪数据
//		{
//			ZGyroModuleAngle = 0.001f * ((int32_t)(rx_message.Data[0] << 24) |
//																						(rx_message.Data[1] << 16) |
//																						(rx_message.Data[2] << 8) |
//																						(rx_message.Data[3]));
//			
//				
//			Gyro_palstance = 0.001f * ((int32_t)(rx_message.Data[4] << 24) |
//																					(rx_message.Data[5] << 16) |
//																					(rx_message.Data[6] << 8) |
//																					(rx_message.Data[7]));
//		}
//		break;	
		
		case UWB_LOCALIZATION_ID://解析uwb信息
		{	
			
		  if(rx_message_DLC_flag == 2 && rx_message.DLC == 8)
			{
				Uwb_data.distance_uwb[1] = rx_message.Data[1] << 8 | rx_message.Data[0];
				Uwb_data.distance_uwb[2] = rx_message.Data[3] << 8 | rx_message.Data[2];
				Uwb_data.distance_uwb[3] = rx_message.Data[5] << 8 | rx_message.Data[4];
				Uwb_data.distance_uwb[4] = rx_message.Data[7] << 8 | rx_message.Data[6]; 
				
				rx_message_DLC_flag = 0;
			}
		  if(rx_message_DLC_flag == 1 && rx_message.DLC == 8)
			{		
				asddwfw ++;
				
				Uwb_data.x_uwb           =  rx_message.Data[1] << 8 | rx_message.Data[0];
				Uwb_data.y_uwb           =  rx_message.Data[3] << 8 | rx_message.Data[2];
				Uwb_data.yaw_angle_uwb   = (rx_message.Data[5] << 8 | rx_message.Data[4]) / 100.0 - Uwb_data.init_yaw_angle_uwb;	
		  	if(Uwb_data.yaw_angle_uwb < 0)
				{
				  Uwb_data.yaw_angle_uwb   = Uwb_data.yaw_angle_uwb + 360;
				}			
				
				Uwb_data.distance_uwb[0] =  rx_message.Data[7] << 8 | rx_message.Data[6]; 
				
				sendtoROS_uwb_flag1 = ~sendtoROS_uwb_flag1;
				rx_message_DLC_flag = 2;
			}
	
			if(rx_message_DLC_flag == 0 && rx_message.DLC == 6)
			{
				Uwb_data.distance_uwb[5]       = rx_message.Data[1] << 8 | rx_message.Data[0];
				Uwb_data.error_type_signal_uwb = rx_message.Data[3] << 8 | rx_message.Data[2];
				Uwb_data.reserve_uwb           = rx_message.Data[5] << 8 | rx_message.Data[4]; 
				
				rx_message_DLC_flag = 1;
			}
		}
		break;	
	}
}
			
	
/////////////////////////////////////////////


//自动任务：
void automatic_control(void)
{
	if(RC_CtrlData.rc.s2 == AUTOMATIC_INPUT)
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
}

void ranger_movement(void) //游荡模式
{
	
//  switch(communication_temp)
//	{
//		case 'p':  //停止
//		{
//			chassis_Motor_M1_set = 0;
//			chassis_Motor_M2_set = 0;
//			chassis_Motor_M3_set = 0;
//			chassis_Motor_M4_set = 0;
//		}break;
//		
//		case 'w':  //前进
//		{
//			chassis_Motor_M1_set = -660 * 0.4;
//			chassis_Motor_M2_set =  660 * 0.4;
//			chassis_Motor_M3_set =  660 * 0.4;
//			chassis_Motor_M4_set = -660 * 0.4;
//		}break;
//		
//		case 's':  //后退
//		{
//			chassis_Motor_M1_set =  660 * 0.4;
//			chassis_Motor_M2_set = -660 * 0.4;
//			chassis_Motor_M3_set = -660 * 0.4;
//			chassis_Motor_M4_set =  660 * 0.4;
//		}break;
//		
//		case 'a':  //左移 97
//		{
//			chassis_Motor_M1_set = -660 * 0.4;
//			chassis_Motor_M2_set = -660 * 0.4;
//			chassis_Motor_M3_set =  660 * 0.4;
//			chassis_Motor_M4_set =  660 * 0.4;
//		}break;
//		
//		case 'd':  //左移 100
//		{
//			chassis_Motor_M1_set =  660 * 0.4;
//			chassis_Motor_M2_set =  660 * 0.4;
//			chassis_Motor_M3_set = -660 * 0.4;
//			chassis_Motor_M4_set = -660 * 0.4;
//		}break;		
//		
//  	case 'q':  //左转 
//		{
//      yaw_angle_set +=       -360 * 0.003;
//		}break;		
//		
//		case 'e':  //右转
//		{
//      yaw_angle_set +=        360 * 0.003;
//		}break;	
//		
//	}
//		
}

float automatic_keep_distance_para = 5;
void attack_movement(void)
{
//	//保持距离：
//  move_for_behind_value = (top_distance - 60) * automatic_keep_distance_para;
//  if(move_for_behind_value >= 100)
//	{
//		move_for_behind_value = 100;
//	}
//	else if(move_for_behind_value <= -100)
//	{
//		move_for_behind_value = -100;
//	}
 
 
}
	