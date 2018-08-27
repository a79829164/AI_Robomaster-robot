#include "usart1.h" 

static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];

void usart1_config(float bound)
{
  GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart1;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef  dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); //使能GPIOA时钟
	
	GPIO_PinAFConfig(GPIOB , GPIO_PinSource7 , GPIO_AF_USART1);
	
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF;	
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB , &gpio);
	
  USART_DeInit(USART1);
  USART_StructInit(&usart1);
	usart1.USART_BaudRate = bound;//波特率设置
	usart1.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	usart1.USART_StopBits = USART_StopBits_1;//一个停止位
	usart1.USART_Parity = USART_Parity_Even;//无奇偶校验位
	usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	usart1.USART_Mode = USART_Mode_Rx  ;	//收发模式
  USART_Init(USART1, &usart1); //初始化串口1
	
	USART_DMACmd(USART1 , USART_DMAReq_Rx , ENABLE);
	
	DMA_DeInit(DMA2_Stream2);
	DMA_StructInit(&dma);
  dma.DMA_Channel = DMA_Channel_4;  
  dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);		//查手册可知DR寄存器偏移地址为0x28
  dma.DMA_Memory0BaseAddr = (uint32_t)&_USART1_DMA_RX_BUF[0][0];
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;					//外设到内存
  dma.DMA_BufferSize = sizeof(_USART1_DMA_RX_BUF)/2;														
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增加
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;				//内存地址增加
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据宽度为32位
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//内存数据宽度16位
  dma.DMA_Mode = DMA_Mode_Circular;													//DMA循环传送
  dma.DMA_Priority = DMA_Priority_Medium;
  dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &dma);	
  
  DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&_USART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
  DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
  DMA_Cmd(DMA2_Stream2, ENABLE);
	
	nvic.NVIC_IRQChannel = USART1_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 2;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART1, ENABLE);

}


//串口接收中断服务函数
void USART1_IRQHandler(void)
{
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












	