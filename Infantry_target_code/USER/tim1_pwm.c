#include "tim1_pwm.h"

 
void tim1_pwm_config(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    NVIC_InitTypeDef         nvic; 
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   //PCLK1=42MHz,TIM2 clk =84MHz
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   //PCLK1=42MHz,TIM4 clk =84MHz

    gpio.GPIO_Pin = GPIO_Pin_0 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);

	  gpio.GPIO_Pin = GPIO_Pin_12 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD,&gpio);
	
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0 , GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12, GPIO_AF_TIM4); 
 
    /* TIM2 */
    tim.TIM_Prescaler = 168 - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000;    
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2,&tim);
		
	  nvic.NVIC_IRQChannel = TIM2_IRQn;	  
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 0;	
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		 
		TIM_ClearFlag(TIM2,TIM_FLAG_Update);
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
		
    /* TIM4 */
		tim.TIM_Prescaler = 90 - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;    
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4,&tim);
//		
//	  nvic.NVIC_IRQChannel = TIM4_IRQn;	  
//		nvic.NVIC_IRQChannelPreemptionPriority = 1;
//		nvic.NVIC_IRQChannelSubPriority = 1;	
//		nvic.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&nvic);
//		
//		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
//		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
//		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
	  oc.TIM_Pulse = 50;
    TIM_OC1Init(TIM2,&oc);
 
		oc.TIM_Pulse = 500;
		TIM_OC1Init(TIM4,&oc);
    
    TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
		TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM2,ENABLE);
    TIM_ARRPreloadConfig(TIM4,ENABLE);
		
    TIM_Cmd(TIM2,ENABLE);
		TIM_Cmd(TIM4,ENABLE);
	 
//  TIM2->CCR1 = 50;      64
//  TIM4->CCR1 = 500;	    770
}

void chassis_task(void);
void shoot_task(void);

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET)
	{
		TIM_ClearFlag(TIM2,TIM_FLAG_Update);

	  shoot_task();
    chassis_task();
	}
}

//void TIM4_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET)
//	{
//		TIM_ClearFlag(TIM4,TIM_FLAG_Update);
//		

//	}
//}

