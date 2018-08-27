#include "usart2.h" 
 
Top_Data_t Top_Data;


char rxbuf[rx_buf_length];// 接收数据的缓冲区
void usart2_config(float bound)
{
  GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart2;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef  dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); //使能GPIOA时钟
	
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource5 , GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD , GPIO_PinSource6 , GPIO_AF_USART2);
		
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD , &gpio);
	
  USART_DeInit(USART2);
  USART_StructInit(&usart2);
	usart2.USART_BaudRate = bound;//波特率设置
	usart2.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	usart2.USART_StopBits = USART_StopBits_1;//一个停止位
	usart2.USART_Parity = USART_Parity_Even;//无奇偶校验位
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  usart2.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;	//收发模式
  USART_Init(USART2, &usart2); //初始化串口1
	
	USART_DMACmd(USART2 , USART_DMAReq_Rx , ENABLE);

	DMA_DeInit(DMA1_Stream5);  
	dma.DMA_Channel = DMA_Channel_4;   
	dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);  
	dma.DMA_Memory0BaseAddr = (uint32_t)rxbuf;  
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	dma.DMA_BufferSize = rx_buf_length;  
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;   
	dma.DMA_Mode = DMA_Mode_Circular;  
	dma.DMA_Priority = DMA_Priority_VeryHigh;  
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;           
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
			 
	DMA_Init(DMA1_Stream5, &dma);    
	DMA_Cmd(DMA1_Stream5,ENABLE);  

	nvic.NVIC_IRQChannel = USART2_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 2;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 1;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART2, ENABLE);

}

////串口接收中断服务函数
extern u8 automatic_control_mode;
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
     	Top_Data.Top_Mode      = (rxbuf[ccnt + 2]  << 8) | rxbuf[ccnt + 1];
			Top_Data.Top_Direction = (rxbuf[ccnt + 4]  << 8) | rxbuf[ccnt + 3];
			Top_Data.Top_Speed     = (rxbuf[ccnt + 6]  << 8) | rxbuf[ccnt + 5];
			Top_Data.Top_Yaw       = (rxbuf[ccnt + 8]  << 8) | rxbuf[ccnt + 7];
			Top_Data.Top_Pitch     = (rxbuf[ccnt + 10] << 8) | rxbuf[ccnt + 9];
			Top_Data.Top_dx        = (rxbuf[ccnt + 12] << 8) | rxbuf[ccnt + 11];
			Top_Data.Top_dy        = (rxbuf[ccnt + 14] << 8) | rxbuf[ccnt + 13];        
			
			automatic_control_mode = Top_Data.Top_Mode;
			top_Direction          = Top_Data.Top_Direction;
			top_Speed              = Top_Data.Top_Speed;
			top_Yaw                = Top_Data.Top_Yaw;
			top_Pitch              = Top_Data.Top_Pitch;
			top_dx                 = Top_Data.Top_dx;
			top_dy                 = Top_Data.Top_dy;
		}
		
    DMA_SetCurrDataCounter(DMA1_Stream5,rx_buf_length);  
    DMA_Cmd(DMA1_Stream5,ENABLE);  
	}
}







