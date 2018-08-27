#include "usart2.h" 
 #include "can2.h"
#include "bsp_imu.h"

Top_Data_t Top_Data;


extern char rxbuf[rx_buf_length];// 接收数据的缓冲区
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
	usart2.USART_Parity = USART_Parity_No;//无奇偶校验位
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

union d
{
	unsigned char bytet[4];
	float num;
	
}floatnum;

extern s16 Yaw_Motor_encoding;
extern uint16_t remain_Hp  ;
extern uint16_t armor_Type ;
extern int shoot_bullet_cnt;
int16_t last_uwb_x , last_uwb_y;
float sendtoROS_uwb_yawangle;
extern u8 sendtoROS_uwb_flag1 , location_point_cnt ;
extern float Bullet_speed;
extern uint16_t  BUFF_attain_flag;

void SendMessage_to_Ros()
{
	if(f(Uwb_data.x_uwb - last_uwb_x) >= 200)
	{
		Uwb_data.x_uwb = last_uwb_x;
	}
	if(f(Uwb_data.y_uwb - last_uwb_y) >= 200)
	{
		Uwb_data.y_uwb = last_uwb_y;
	}
	tx_buf_toROS[0]  = 0x7F; 
	//裁判系统
	tx_buf_toROS[1]  = remain_Hp ;
	tx_buf_toROS[2]  = remain_Hp >> 8; 
	tx_buf_toROS[3]  = armor_Type ;
	tx_buf_toROS[4]  = armor_Type >> 8;
	//已发射的子弹
	tx_buf_toROS[5]  = shoot_bullet_cnt;
	tx_buf_toROS[6]  = shoot_bullet_cnt >> 8;
	//uwb坐标和方向角：
	tx_buf_toROS[7]  = Uwb_data.x_uwb ;
	tx_buf_toROS[8]  = Uwb_data.x_uwb >> 8;
	tx_buf_toROS[9]  = Uwb_data.y_uwb ;
	tx_buf_toROS[10] = Uwb_data.y_uwb >> 8;
	
	sendtoROS_uwb_yawangle = Uwb_data.yaw_angle_uwb;
	if(sendtoROS_uwb_yawangle < 0)
	{
		sendtoROS_uwb_yawangle = sendtoROS_uwb_yawangle + 360;
	}
	floatnum.num = sendtoROS_uwb_yawangle;
	
//	tx_buf_toROS[11] = (int16_t)(sendtoROS_uwb_yawangle * 100); 
//	tx_buf_toROS[12] = (int16_t)(sendtoROS_uwb_yawangle * 100) >> 8; 
	//	//pitch角：
	tx_buf_toROS[11] = (int16_t)((1.0 * (Pitch_Motor[0] - Pitch_horizontal_level) / 8192 * 360) * 100); 
	tx_buf_toROS[12] = (int16_t)((1.0 * (Pitch_Motor[0] - Pitch_horizontal_level) / 8192 * 360) * 100) >> 8; 

	tx_buf_toROS[13] = floatnum.bytet[0];
	tx_buf_toROS[14] = floatnum.bytet[1];
	tx_buf_toROS[15] = floatnum.bytet[2];
	tx_buf_toROS[16] = floatnum.bytet[3];

	
//	
//	//云台与底盘夹角：
//	tx_buf_toROS[13] = (int16_t)((1.0 * (Yaw_Motor_encoding - Yaw_motorloco_init) / 8192 * 360) * 100); 
//	tx_buf_toROS[14] = (int16_t)((1.0 * (Yaw_Motor_encoding - Yaw_motorloco_init) / 8192 * 360) * 100) >> 8;
//	//pitch角：
//	tx_buf_toROS[15] = (int16_t)((1.0 * (Pitch_Motor[0] - Pitch_horizontal_level) / 8192 * 360) * 100); 
//	tx_buf_toROS[16] = (int16_t)((1.0 * (Pitch_Motor[0] - Pitch_horizontal_level) / 8192 * 360) * 100) >> 8; 

	//imu 六轴数据：
	tx_buf_toROS[17] = (int16_t)(mpu_data.ax)  ; 
	tx_buf_toROS[18] = (int16_t)(mpu_data.ax) >> 8; 
	tx_buf_toROS[19] = (int16_t)(mpu_data.ay) ; 
	tx_buf_toROS[20] = (int16_t)(mpu_data.ay) >> 8; 
	tx_buf_toROS[21] = (int16_t)(mpu_data.az) ; 
	tx_buf_toROS[22] = (int16_t)(mpu_data.az) >> 8; 
	tx_buf_toROS[23] = (int16_t)(imu.wx * 10000) ; 
	tx_buf_toROS[24] = (int16_t)(imu.wx * 10000) >> 8; 
	tx_buf_toROS[25] = (int16_t)(imu.wy * 10000) ; 
	tx_buf_toROS[26] = (int16_t)(imu.wy * 10000) >> 8; 
	tx_buf_toROS[27] = (int16_t)(imu.wz * 10000) ; 
	tx_buf_toROS[28] = (int16_t)(imu.wz * 10000) >> 8; 
  //轮式里程计：	X Y
	tx_buf_toROS[29] = (int16_t)(speed_x * 10000) ; 
	tx_buf_toROS[30] = (int16_t)(speed_x * 10000)>> 8; // X
	tx_buf_toROS[31] = (int16_t)(speed_y * 10000); 
	tx_buf_toROS[32] = (int16_t)(speed_y * 10000)>> 8; // Y
	tx_buf_toROS[33] = sendtoROS_uwb_flag1;
	//////
	tx_buf_toROS[34] = location_point_cnt;
	//
	tx_buf_toROS[35] = (int16_t)(Bullet_speed * 1000);
	tx_buf_toROS[36] = (int16_t)(Bullet_speed * 1000) >> 8;
	
	tx_buf_toROS[37] = BUFF_attain_flag;
	tx_buf_toROS[38] = BUFF_attain_flag >> 8;	
	
	//	//云台与底盘夹角：
	tx_buf_toROS[39] = (int16_t)((1.0 * (Yaw_Motor[0] - Yaw_motorloco_init) / 8192 * 360) * 100); 
	tx_buf_toROS[40] = (int16_t)((1.0 * (Yaw_Motor[0] - Yaw_motorloco_init) / 8192 * 360) * 100) >> 8; 
// 
	tx_buf_toROS[41] = 0x7E  ;
	for(int i = 0 ; i < 42 ; i++)
	{
		USART_SendData(USART2 , tx_buf_toROS[i]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC ) != SET);
	}
	last_uwb_x = Uwb_data.x_uwb;
	last_uwb_y = Uwb_data.y_uwb;
}




