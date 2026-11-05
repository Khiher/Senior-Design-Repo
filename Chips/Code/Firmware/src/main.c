/**
 * NUCLEO-L152RE -> TI DAC7562 (DACxx6x family) over SPI1.
 *
 * Wiring (Arduino header):
 *   DAC SCLK  (pin 7) <- PB3 (D3)   SPI1_SCK  (remapped; PA5/D13 is the LD2 LED)
 *   DAC DIN   (pin 8) <- PB5 (D4)   SPI1_MOSI
 *   DAC SYNC  (pin 6) <- PB6 (D10)  GPIO, software chip select
 *   DAC LDAC  (pin 4) <- PC7 (D9)   GPIO (or tie to GND)
 *   DAC CLR   (pin 5) <- PA9 (D8)   GPIO (or tie to AVDD)
 *   DAC AVDD  (pin 9) <- 3V3, GND (pin 3) <- GND, 150 nF on VREF (pin 10) if using internal ref
 *
 * Demo: triangle wave on VOUTA, inverted triangle on VOUTB. LD2 (PA5) blinks.
 */
#include "stm32l1xx_hal.h"
#include "dac856x.h"

#define DAC_RESOLUTION   12   /* 12 = DAC756x (DAC7562), 14 = DAC816x, 16 = DAC856x */
#define USE_INTERNAL_REF 1    /* 1: 2.5 V internal ref, gain 2 -> 0..5 V (clipped at AVDD) */

SPI_HandleTypeDef hspi1;

static dac856x_t dac = {
    .hspi = &hspi1,
    .sync_port = GPIOB, .sync_pin = GPIO_PIN_6,
    .ldac_port = GPIOC, .ldac_pin = GPIO_PIN_7,
    .clr_port  = GPIOA, .clr_pin  = GPIO_PIN_9,
    .resolution = DAC_RESOLUTION,
};

static void clock_init(void);
static void gpio_init(void);
static void spi1_init(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    clock_init();        /* 32 MHz from HSI + PLL */
    gpio_init();
    spi1_init();

    if (dac_init(&dac, USE_INTERNAL_REF) != HAL_OK) Error_Handler();

    const uint16_t max_code = (uint16_t)((1u << DAC_RESOLUTION) - 1u);
    const uint16_t step = (uint16_t)(max_code / 256u);
    uint32_t code = 0;
    int dir = 1;
    uint32_t last_blink = 0;

    while (1) {
        dac_set(&dac, DAC_CH_A, (uint16_t)code);
        dac_set(&dac, DAC_CH_B, (uint16_t)(max_code - code));

        if (dir > 0) {
            code += step;
            if (code >= max_code) { code = max_code; dir = -1; }
        } else {
            code = (code > step) ? code - step : 0;
            if (code == 0) dir = 1;
        }

        if (HAL_GetTick() - last_blink >= 500) {
            last_blink = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
        HAL_Delay(1);
    }
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* 32 MHz requires voltage range 1 */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLMUL = RCC_PLL_MUL6;   /* 16 MHz * 6 = 96 MHz VCO */
    osc.PLL.PLLDIV = RCC_PLL_DIV3;   /* 96 / 3 = 32 MHz SYSCLK */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Idle levels: SYNC, LDAC, CLR high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &g);

    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_5;  /* LD2 */
    HAL_GPIO_Init(GPIOA, &g);
}

static void spi1_init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;      /* mode 1: shift on rising,    */
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;          /* DAC samples on falling edge */
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; /* 32 MHz APB2 / 4 = 8 MHz (DAC max 50 MHz) */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

/* Called by HAL_SPI_Init */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    GPIO_InitTypeDef g = {0};
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin = GPIO_PIN_3 | GPIO_PIN_5;  /* PB3 SCK, PB5 MOSI */
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &g);
}

void Error_Handler(void)
{
    /* LD2 fast blink */
    __disable_irq();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        for (volatile uint32_t i = 0; i < 100000; i++) { }
    }
}
