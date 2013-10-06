/*
 * Module responsible for communicetion using uart
 */

#ifndef __H_DEBUG_H__
#define __H_DEBUG_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"



void debug_Init(void);
void debug_Print(const char *msg);
void debug_ParseIncoming(void);

#endif

