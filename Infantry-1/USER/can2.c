#include "can2.h"


Uwb_data_t Uwb_data;

void Can2_Configuration(void)
{
	CAN_InitTypeDef can2;
	GPIO_InitTypeDef       gpio;
	CAN_FilterInitTypeDef  can_filter;
	NVIC_InitTypeDef       nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &gpio);
	
	GPIO_PinAFConfig(GPIOB , GPIO_PinSource12 , GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB , GPIO_PinSource13 , GPIO_AF_CAN2);
	
	can2.CAN_TTCM = DISABLE;
	can2.CAN_ABOM = DISABLE;
	can2.CAN_AWUM = DISABLE;
	can2.CAN_NART = DISABLE;
	can2.CAN_RFLM = DISABLE;
	can2.CAN_TXFP = ENABLE;
	can2.CAN_Mode = CAN_Mode_Normal;
	can2.CAN_SJW  = CAN_SJW_1tq;
	can2.CAN_BS1 = CAN_BS1_9tq;
	can2.CAN_BS2 = CAN_BS2_4tq;
	can2.CAN_Prescaler = 3;        
	CAN_Init(CAN2, &can2); //  180M / 4 = 45M , 45M / 3 = 15M , 15M / 
 
	can_filter.CAN_FilterNumber=14;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;	
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;	
	can_filter.CAN_FilterFIFOAssignment=0;		
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);

	nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 3;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);	
	CAN_ITConfig(CAN2,CAN_IT_TME ,ENABLE);		
}

void Send_RMGyro_Reset(void)
{
	CanTxMsg tx_message;
	tx_message.StdId = 0x406;							//报文ID号
	tx_message.RTR = CAN_RTR_Data;				//数据帧
	tx_message.IDE = CAN_Id_Standard;			//标准ID
	tx_message.DLC = 0x08;								//数据字节数

  tx_message.Data[0] = 0;
  tx_message.Data[1] = 1  ;
  tx_message.Data[2] = 2;
  tx_message.Data[3] = 3  ;
  tx_message.Data[4] = 4;
  tx_message.Data[5] = 5  ;
  tx_message.Data[6] = 6;
  tx_message.Data[7] = 7  ;
	
	CAN_Transmit(CAN2,&tx_message);
}	



	