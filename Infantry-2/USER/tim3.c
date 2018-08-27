#include "tim3.h"


void Tim3_config(void)
{
	TIM_TimeBaseInitTypeDef  tim;
  NVIC_InitTypeDef         nvic; 
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
 
  nvic.NVIC_IRQChannel = TIM3_IRQn;	  
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;	
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	tim.TIM_Prescaler = 90 - 1;
	tim.TIM_Period = 1000;
	tim.TIM_ClockDivision = 0 ;
	tim.TIM_CounterMode=TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM3,&tim);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3, ENABLE);										
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);	
}


void Tim5_config(void)
{
	TIM_TimeBaseInitTypeDef  tim;
  NVIC_InitTypeDef         nvic; 
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
 
  nvic.NVIC_IRQChannel = TIM5_IRQn;	  
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 3;	
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	tim.TIM_Prescaler = 90 - 1;
	tim.TIM_Period = 1000;
	tim.TIM_ClockDivision = 0 ;
	tim.TIM_CounterMode=TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM5,&tim);
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5, ENABLE);										
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);	
}



