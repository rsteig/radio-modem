/*
 * Module responsible for handling buttons and diodes
 */

#ifndef __H_PORTIO_H__
#define __H_PORTIO_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define PORTIO_OFF      0
#define PORTIO_ON       1

#define PORTIO_LED_R GPIO_Pin_8
#define PORTIO_LED_G GPIO_Pin_9



void portio_Init(void);
void portio_Led(uint16_t led, int8_t status);



#endif

