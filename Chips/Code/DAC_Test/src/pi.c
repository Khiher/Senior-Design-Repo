#include "pi.h"
#include "stm32l1xx_hal.h"

/* Pulse timing. 100 MHz clock -> 10 ns period; 500 ns gives the chip's synchronizer
 * 50 clocks of margin. Interrupts can only stretch these, which is harmless for an
 * edge-triggered input. */
#define PULSE_HIGH_NS  500u
#define PULSE_LOW_NS   500u

typedef struct { GPIO_TypeDef *port; uint16_t pin; } pin_t;

static const pin_t up_pin[PI_COUNT] = { { GPIOB, GPIO_PIN_10 }, { GPIOA, GPIO_PIN_9 } };
static const pin_t dn_pin[PI_COUNT] = { { GPIOA, GPIO_PIN_8 },  { GPIOC, GPIO_PIN_7 } };

static int32_t position[PI_COUNT];

static void delay_ns(uint32_t ns)
{
    uint32_t cycles = (SystemCoreClock / 1000000u) * ns / 1000u;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { }
}

static void pulse(const pin_t *p, uint32_t n)
{
    while (n--) {
        p->port->BSRR = p->pin;                       /* high */
        delay_ns(PULSE_HIGH_NS);
        p->port->BSRR = (uint32_t)p->pin << 16;       /* low  */
        delay_ns(PULSE_LOW_NS);
    }
}

void pi_init(void)
{
    GPIO_InitTypeDef g = {0};

    /* Cycle counter for ns delays */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    for (int i = 0; i < PI_COUNT; i++) {
        HAL_GPIO_WritePin(up_pin[i].port, up_pin[i].pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dn_pin[i].port, dn_pin[i].pin, GPIO_PIN_RESET);
        g.Pin = up_pin[i].pin; HAL_GPIO_Init(up_pin[i].port, &g);
        g.Pin = dn_pin[i].pin; HAL_GPIO_Init(dn_pin[i].port, &g);
        position[i] = 0;
    }
}

void pi_up(int pi, uint32_t n)
{
    if (pi < 1 || pi > PI_COUNT) return;
    pulse(&up_pin[pi - 1], n);
    position[pi - 1] += (int32_t)n;
}

void pi_down(int pi, uint32_t n)
{
    if (pi < 1 || pi > PI_COUNT) return;
    pulse(&dn_pin[pi - 1], n);
    position[pi - 1] -= (int32_t)n;
}

void pi_reset(int pi)
{
    if (pi < 1 || pi > PI_COUNT) return;
    pulse(&dn_pin[pi - 1], PI_RESET_PULSES);
    position[pi - 1] = 0;
}

int32_t pi_position(int pi)
{
    return (pi < 1 || pi > PI_COUNT) ? 0 : position[pi - 1];
}
