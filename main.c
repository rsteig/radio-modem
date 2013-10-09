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
#include "cc1000.h"

void TxTest(void)
{
    cc1000_SetModeTx();

    while (1) {
        cc1000_SendData((int8_t *)("DUPA"), 4);
        delay_MsBlockWait(500, DELAY_TIMER_MAIN);
    }
}

void RxTest(void)
{
    cc1000_SetModeRx();

    while (1) {
        if (cc1000_IsDataReceived()) {
            delay_MsBlockWait(100, DELAY_TIMER_MAIN);
            cc1000_ClearRxFlag();
        }
    }

}

void main_Init();

int main(void)
{
    uint8_t i = 0;

    main_Init();    //init everything

    for (i = 0; i < 3; ++i) {
        debug_Print("Blink...");

        portio_Led(PORTIO_LED_RX, PORTIO_ON);
        portio_Led(PORTIO_LED_TX, PORTIO_ON);
        delay_MsBlockWait(1000, DELAY_TIMER_MAIN);

        portio_Led(PORTIO_LED_RX, PORTIO_OFF);
        portio_Led(PORTIO_LED_TX, PORTIO_OFF);
        delay_MsBlockWait(1000, DELAY_TIMER_MAIN);
    }

    //TxTest();
    RxTest();

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
    cc1000_Init();
}

