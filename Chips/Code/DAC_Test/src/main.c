/**
 * NUCLEO-L152RE internal 12-bit DAC: fixed DC levels.
 *
 *   DAC_A = DAC_OUT1 = PA4 -> Arduino A2  (CN8 pin 3)  / Morpho CN7 pin 32   1.6 V
 *   DAC_B = DAC_OUT2 = PA5 -> Arduino D13 (CN5 pin 6)  / Morpho CN10 pin 11  0.2 V
 *   Ground: any GND pin.
 *
 * VOUT = VDDA * code / 4095, with VDDA = 3.3 V on the Nucleo (VREF+ is tied to VDDA
 * on LQFP64). Buffered output range is 0.2 V .. VDDA - 0.2 V.
 */
#include "stm32l1xx_hal.h"

#define VDDA_MV      3300u
#define DAC_A_MV     1200u
#define DAC_B_MV     200u
#define MV_TO_CODE(mv)  (((mv) * 4095u + VDDA_MV / 2u) / VDDA_MV)  /* 1.6 V -> 1985, 0.2 V -> 248 */

DAC_HandleTypeDef hdac;

static void clock_init(void);
static void dac_init(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    clock_init();
    dac_init();

    while (1) { }
}

static void dac_init(void)
{
    DAC_ChannelConfTypeDef ch = {0};

    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK) Error_Handler();

    ch.DAC_Trigger = DAC_TRIGGER_NONE;
    ch.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &ch, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_DAC_ConfigChannel(&hdac, &ch, DAC_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, MV_TO_CODE(DAC_A_MV));
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, MV_TO_CODE(DAC_B_MV));
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
}

/* Called by HAL_DAC_Init */
void HAL_DAC_MspInit(DAC_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_DAC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) { }  /* required, see RM0038 */

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLMUL = RCC_PLL_MUL6;   /* 96 MHz VCO */
    osc.PLL.PLLDIV = RCC_PLL_DIV3;   /* 32 MHz SYSCLK */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
