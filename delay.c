#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "delay.h"



volatile uint32_t delay_timer[DELAY_MAX_TIMERS];
uint8_t test;



void delay_Init(void)
{
    uint8_t i = 0;

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
   
    //clearing timers
    for (i = 0; i < DELAY_MAX_TIMERS; i++)
        delay_timer[i] = 0;
   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
   
    //NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStructure);
    

    TIM_TimeBaseStructure.TIM_Period = 999;  //1kHz
    TIM_TimeBaseStructure.TIM_Prescaler = 32;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
    

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 999;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
    
    TIM_Cmd(TIM1, ENABLE);
}

void delay_ClearTimer(uint8_t timer)
{
    delay_timer[timer] = 0;
}

uint32_t delay_GetTimer(uint8_t timer)
{
    return delay_timer[timer];
}

void delay_MsBlockWait(uint32_t time, uint8_t timer)
{
    delay_timer[timer] = 0;

    while (delay_timer[timer] < time);
}



void TIM1_CC_IRQHandler(void)
{
    uint8_t i = 0;

    if (TIM_GetITStatus(TIM1,TIM_IT_CC1) == SET) {
        for (i = 0; i < DELAY_MAX_TIMERS; ++i) {
            ++delay_timer[i];
    
            if(delay_timer[i] > 0xfffffffa)
                delay_timer[i] = 0xfffffffa;
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
    }
}

