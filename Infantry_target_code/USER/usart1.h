#include "stm32f4xx.h" 
#include "remote_task.h"




#define  BSP_USART1_DMA_RX_BUF_LEN               30u                   
 
#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES         (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define  RC_FRAME_LENGTH                            18u

void usart1_config(float bound);


































