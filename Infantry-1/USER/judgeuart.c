#include "judgeuart.h" 
#include "string.h"

uint8_t judge_buf[judge_bux_rx];// 接收数据的缓冲区
void usart6_config(void)
{
  GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef  dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); //使能GPIOA时钟
	
	GPIO_PinAFConfig(GPIOG , GPIO_PinSource9  , GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG , GPIO_PinSource14 , GPIO_AF_USART6);
		
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG , &gpio);
	
  USART_DeInit(USART6);
  USART_StructInit(&usart);
	usart.USART_BaudRate = 115200;//波特率设置
	usart.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	usart.USART_StopBits = USART_StopBits_1;//一个停止位
	usart.USART_Parity = USART_Parity_No;//无奇偶校验位
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;	//收发模式
  USART_Init(USART6, &usart); //初始化串口1
	
	USART_DMACmd(USART6 , USART_DMAReq_Rx , ENABLE);

	DMA_DeInit(DMA2_Stream1);  
	dma.DMA_Channel = DMA_Channel_5;   
	dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  
	dma.DMA_Memory0BaseAddr = (uint32_t)judge_buf;  
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	dma.DMA_BufferSize = judge_bux_rx;  
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;   
	dma.DMA_Mode = DMA_Mode_Circular;  
	dma.DMA_Priority = DMA_Priority_Low;  
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;           
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
			 
	DMA_Init(DMA2_Stream1, &dma);    
	DMA_Cmd(DMA2_Stream1,ENABLE);  

	nvic.NVIC_IRQChannel = USART6_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 3;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 1;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART6, ENABLE);
}

 

//串口接收中断服务函数
void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART6->SR;
		(void)USART6->DR;

    DMA_Cmd(DMA2_Stream1 , DISABLE);    
		judgement_data_handle();		
		DMA_SetCurrDataCounter(DMA2_Stream1 , judge_bux_rx);
    DMA_Cmd(DMA2_Stream1 , ENABLE);  
		
	}
}

receive_judge_t judge_rece_mesg;
uint8_t cmd_id ;

int16_t  judgeuart_run_cnt = 0;

float Bullet_speed;
float Bullet_freq;
uint16_t GetICRA_Buff_info;
uint16_t  cardtype , cardidx , rfid_detect_cnt = 0 , rfid_detect_last_cnt = 0;

uint16_t remain_Hp = 2000;
uint16_t armor_Type = 0;

int armor_attacked_cnt = 0;

uint16_t  BUFF_attain_flag = 0;

int UWB_X , UWB_Y , UWB_Z;
void judgement_data_handle(void)
{
  frame_header_t *frame_header = (frame_header_t*)judge_buf;
  memcpy(frame_header, judge_buf, HEADER_LEN);
	cmd_id = *(uint16_t *)(judge_buf + HEADER_LEN);
	uint16_t data_length = frame_header->data_length;
	uint8_t *data_addr   = judge_buf + HEADER_LEN + CMD_LEN;
	
	if(frame_header->sof == SOF_FIXED)
	{
//		if(cmd_id == REAL_FIELD_DATA_ID && rfid_detect_cnt == 0)
//		{
//			judgeuart_run_cnt++;
//		}
//	  judgeuart_run_cnt++;
		switch(cmd_id)
		{
			case GAME_INFO_ID:
				memcpy(&judge_rece_mesg.game_information  , data_addr , data_length);
			  remain_Hp = judge_rece_mesg.game_information.remainHP;
			break;

			case REAL_BLOOD_DATA_ID:
				memcpy(&judge_rece_mesg.blood_changed_data, data_addr , data_length);
 			  armor_Type = judge_rece_mesg.blood_changed_data.armor_type;
			  armor_attacked_cnt++;
        remain_Hp = remain_Hp - 50;
			break;
			
 			case REAL_SHOOT_DATA_ID:
        memcpy(&judge_rece_mesg.real_shoot_data   , data_addr , data_length);
  			Bullet_speed = judge_rece_mesg.real_shoot_data.bulletSpeed;
		  	Bullet_freq  = judge_rece_mesg.real_shoot_data.bulletFreq;
      break;
			
			case REAL_FIELD_DATA_ID:
        memcpy(&judge_rece_mesg.rfid_data         , data_addr , data_length);
        cardtype = judge_rece_mesg.rfid_data.cardType;
			  cardidx  = judge_rece_mesg.rfid_data.cardIdx;
			  rfid_detect_cnt++;
      break;
			
			case GAIN_BUFF_ID:
        memcpy(&judge_rece_mesg.get_buff_data     , data_addr, data_length);
			  GetICRA_Buff_info = judge_rece_mesg.get_buff_data.buffMusk;
			  
      break;
			
			case UWB_GAIN_ID:
        memcpy(&judge_rece_mesg.game_robot_postion_data     , data_addr, data_length);
			  UWB_X = judge_rece_mesg.game_robot_postion_data.x;
			  UWB_Y = judge_rece_mesg.game_robot_postion_data.y;
		   	UWB_Z = judge_rece_mesg.game_robot_postion_data.z;
      break;
		}
	}
	

 
}







