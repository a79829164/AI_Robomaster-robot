#include "can1.h"

void Can1_Configuration(void)
{
	CAN_InitTypeDef can1;
	GPIO_InitTypeDef       gpio;
	CAN_FilterInitTypeDef  can_filter;
	NVIC_InitTypeDef       nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &gpio);
	
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource0 , GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource1 , GPIO_AF_CAN1);
	
	can1.CAN_TTCM = DISABLE;
	can1.CAN_ABOM = DISABLE;
	can1.CAN_AWUM = DISABLE;
	can1.CAN_NART = ENABLE;
	can1.CAN_RFLM = DISABLE;
	can1.CAN_TXFP = ENABLE;
	can1.CAN_Mode = CAN_Mode_Normal;
	can1.CAN_SJW  = CAN_SJW_1tq;
	can1.CAN_BS1 = CAN_BS1_9tq;
	can1.CAN_BS2 = CAN_BS2_4tq;
	can1.CAN_Prescaler = 3;        
	CAN_Init(CAN1, &can1); //  180M / 4 = 45M , 45M / 3 = 15M , 15M / 
 
	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;	
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;	
	can_filter.CAN_FilterFIFOAssignment=0;		
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);

	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);	
	CAN_ITConfig(CAN1,CAN_IT_TME ,ENABLE);		
}

void Send_motorvalue_CAN1_4(s16 data1 , s16 data2 , s16 data3 , s16 data4)
{
	CanTxMsg tx_message;
	tx_message.StdId = 0x200;							//报文ID号
	tx_message.RTR = CAN_RTR_Data;				//数据帧
	tx_message.IDE = CAN_Id_Standard;			//标准ID
	tx_message.DLC = 0x08;								//数据字节数

  tx_message.Data[0] = data1 >> 8;
  tx_message.Data[1] = data1  ;
  tx_message.Data[2] = data2 >> 8;
  tx_message.Data[3] = data2  ;
  tx_message.Data[4] = data3 >> 8;
  tx_message.Data[5] = data3  ;
  tx_message.Data[6] = data4 >> 8;
  tx_message.Data[7] = data4  ;
	
	CAN_Transmit(CAN1,&tx_message);
}
	
void Send_motorvalue_CAN1_3(s16 data1 , s16 data2 , s16 data3 , s16 data4)
{
	CanTxMsg tx_message;
	tx_message.StdId = 0x1ff	;					//报文ID号
	tx_message.RTR = CAN_RTR_Data;				//数据帧
	tx_message.IDE = CAN_Id_Standard;			//标准ID
	tx_message.DLC = 0x08;								//数据字节数

  tx_message.Data[0] = data1 >> 8;
  tx_message.Data[1] = data1  ;
  tx_message.Data[2] = data2 >> 8;
  tx_message.Data[3] = data2  ;
  tx_message.Data[4] = data3 >> 8;
  tx_message.Data[5] = data3  ;
  tx_message.Data[6] = data4 >> 8;
  tx_message.Data[7] = data4  ;
	
	CAN_Transmit(CAN1,&tx_message);
}

	
	
	
	
	
	
	
	
	
	


