#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "portio.h"


void portio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    //init for diodes
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = PORTIO_LED_TX | PORTIO_LED_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void portio_Led(uint16_t led, int8_t action)
{
    if(action)
        GPIO_SetBits(GPIOB, led);
    else
        GPIO_ResetBits(GPIOB, led);
}

