/*
 * Servo driver controled by USB. It uses STM32 microcontroller to controll
 * up to 8 servomotors.
 *
 * Author: Pawel Polawski
 * Email: pawel.polawski@gmail.com
 *
 * */

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "debug.h"
#include "portio.h"
#include "delay.h"



void main_Init();

int main(void)
{
    uint8_t i = 0;

    main_Init();    //init everything

    for (i = 0; i < 10; ++i) {
        debug_Print("Blink...");

        portio_Led(PORTIO_LED_R, PORTIO_ON);
        delay_MsBlockWait(1000, DEALY_TIMER0);

        portio_Led(PORTIO_LED_R, PORTIO_OFF);
        delay_MsBlockWait(1000, DEALY_TIMER0);
    }

    //chcking for input
    while(1) {
        debug_ParseIncoming();
    }

    while(1);
    return 0;
}

void rcc_Init(void)
{
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);

    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS) {
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        //for 36MHz latency is 1
        FLASH_SetLatency(FLASH_Latency_1);

        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        //clock settings for system bus
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div1);

        //4 * 8MHz equal almost 36MHz
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_4);
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08);
    }
}

void main_Init(void)
{
    rcc_Init();

    debug_Init();
    portio_Init();
    delay_Init();
}

