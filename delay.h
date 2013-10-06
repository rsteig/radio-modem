/*
 * Module responsible for generating defined delays.
 */

#ifndef __H_DELAY_H__
#define __H_DELAY_H__



#define DELAY_MAXTIMERS 5

#define DEALY_TIMER0    0
#define DEALY_TIMER1    1
#define DEALY_TIMER2    2
#define DEALY_TIMER3    3
#define DEALY_TIMER4    4



void delay_Init(void);

void delay_ClearTimer(uint8_t timer);
uint32_t delay_GetTimer(uint8_t timer);

void delay_MsBlockWait(uint32_t time, uint8_t timer);



#endif

