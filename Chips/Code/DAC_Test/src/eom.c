#include "eom.h"
#include "stm32l1xx_hal.h"

#define EOM_CLK_FALLING   1     /* 1: count falling edges of EOM_CLK, 0: rising */
#define EOM_ERR_ACTIVE_HI 1     /* 1: EOM_ERR high = error, 0: low = error */

/* TIM2/TIM4 are 16-bit. A window is split into chunks short enough that neither
 * counter can wrap: HAL_Delay(8) lasts 8-9 ms -> <= 56250 edges at 6.25 MHz. */
#define CHUNK_MS          8u

void eom_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLDOWN;      /* defined level if the board isn't connected */
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;          /* PA0 TIM2_ETR, PA1 TIM2_CH2 */
    g.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_6;                       /* PB6 TIM4_CH1 */
    g.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &g);

    /* ---- TIM2: error-cycle counter (ETR clock, gated by TI2 = EOM_ERR) ---- */
    TIM2->CR1   = 0;
    TIM2->PSC   = 0;
    TIM2->ARR   = 0xFFFF;
    TIM2->CCMR1 = TIM_CCMR1_CC2S_0;                          /* IC2 <- TI2, no filter */
    TIM2->CCER  = EOM_ERR_ACTIVE_HI ? 0 : TIM_CCER_CC2P;     /* gate polarity */
    TIM2->SMCR  = TIM_SMCR_ECE                               /* external clock mode 2 */
                | (EOM_CLK_FALLING ? TIM_SMCR_ETP : 0)       /* ETR edge */
                | (TIM_SMCR_TS_2 | TIM_SMCR_TS_1)            /* TS = 110: TI2FP2 */
                | (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0);         /* SMS = 101: gated */
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->SR    = 0;

    /* ---- TIM4: total-cycle counter (external clock mode 1 on TI1 = EOM_CLK) ---- */
    TIM4->CR1   = 0;
    TIM4->PSC   = 0;
    TIM4->ARR   = 0xFFFF;
    TIM4->CCMR1 = TIM_CCMR1_CC1S_0;                          /* IC1 <- TI1, no filter */
    TIM4->CCER  = EOM_CLK_FALLING ? TIM_CCER_CC1P : 0;       /* same edge as TIM2 */
    TIM4->SMCR  = (TIM_SMCR_TS_2 | TIM_SMCR_TS_0)            /* TS = 101: TI1FP1 */
                | TIM_SMCR_SMS;                              /* SMS = 111: ext clock 1 */
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->SR    = 0;
}

eom_status_t eom_measure(uint32_t ms, uint32_t *errors, uint32_t *total)
{
    uint32_t err_sum = 0, tot_sum = 0;
    eom_status_t st = EOM_OK;

    if (ms == 0) ms = 1;

    while (ms) {
        uint32_t chunk = (ms > CHUNK_MS) ? CHUNK_MS : ms;
        ms -= chunk;

        __disable_irq();
        TIM2->CNT = 0;  TIM4->CNT = 0;
        TIM2->SR = 0;   TIM4->SR = 0;
        /* Start and stop in the same order so the few-instruction skew cancels */
        TIM2->CR1 = TIM_CR1_CEN;
        TIM4->CR1 = TIM_CR1_CEN;
        __enable_irq();

        HAL_Delay(chunk - 1u);           /* HAL_Delay(n) waits n..n+1 ms */

        __disable_irq();
        TIM2->CR1 = 0;
        TIM4->CR1 = 0;
        __enable_irq();

        if ((TIM2->SR | TIM4->SR) & TIM_SR_UIF) st = EOM_OVERFLOW;
        err_sum += TIM2->CNT;
        tot_sum += TIM4->CNT;
    }

    *errors = err_sum;
    *total  = tot_sum;
    if (st == EOM_OK && tot_sum == 0) st = EOM_NO_CLOCK;
    return st;
}

int eom_err_level(void)
{
    int lvl = (GPIOA->IDR & GPIO_PIN_1) ? 1 : 0;   /* IDR still reads in AF mode */
    return EOM_ERR_ACTIVE_HI ? lvl : !lvl;
}
