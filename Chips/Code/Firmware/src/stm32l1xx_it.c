#include "stm32l1xx_hal.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    while (1) { }
}
