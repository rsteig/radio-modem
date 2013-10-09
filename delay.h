/*
 * Module responsible for generating defined delays.
 */

#ifndef __H_DELAY_H__
#define __H_DELAY_H__



#define DELAY_MAX_TIMERS 5

#define DELAY_TIMER_0            0
#define DELAY_TIMER_CC1000      1



void delay_Init(void);

void delay_ClearTimer(uint8_t timer);
uint32_t delay_GetTimer(uint8_t timer);

void delay_MsBlockWait(uint32_t time, uint8_t timer);



#endif

