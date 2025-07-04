/*
 * delay.c
 *
 *  Created on: Jul 2, 2025
 *      Author: PANKAJA
 */


#include "main.h"
#include "delay.h"

void Delay_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

// Delays for us microseconds
void delayMicroseconds(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT,
    delayTicks = us * 180; //  MCU runs at 180 MHz, so each microsecond lasts 180 clock ticks

    while (DWT->CYCCNT - startTick < delayTicks);
}
