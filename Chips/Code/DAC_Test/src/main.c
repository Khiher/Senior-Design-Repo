/**
 * NUCLEO-L152RE chip controller: DAC bias levels, phase-interpolator stepping,
 * EOM error counting, and a text command interface for a host GUI.
 *
 * Pinout (Arduino header names; all at 3.3 V, level shifters on the chip board):
 *   DAC_A    PA4  A2    analog out, 1.6 V default
 *   DAC_B    PA5  D13   analog out, 0.2 V default (also LED LD2, which is unused)
 *   PI1_UP   PB10 D6    digital out
 *   PI1_DN   PA8  D7    digital out
 *   PI2_UP   PA9  D8    digital out
 *   PI2_DN   PC7  D9    digital out
 *   EOM_ERR  PA1  A1    digital in  (TIM2 gate)
 *   EOM_CLK  PA0  A0    digital in  (TIM2 ETR)   } wire EOM_CLK to BOTH pins
 *            PB6  D10                (TIM4 CH1)  }
 *   Host     PA2/PA3    USART2 -> ST-LINK virtual COM port, 115200 8N1
 *
 * Startup: clocks -> PI outputs low -> wait for the chip -> 8 DN pulses on each PI
 *          -> DAC levels -> EOM counters -> "READY" on the serial port.
 *
 * Commands (one per line, case-insensitive, replies start with OK or ERR):
 *   ID                  OK CHIPS-CTRL 1.0
 *   STATUS              OK PI1=<pos> PI2=<pos> DACA=<mV> DACB=<mV> ERR=<0|1>
 *   UP <1|2> [n]        pulse PI up n times (default 1)       -> OK PI<x>=<pos>
 *   DN <1|2> [n]        pulse PI down n times (default 1)     -> OK PI<x>=<pos>
 *                       Each PI has 8 positions (0..7). Moves stop at the ends, so the
 *                       reply may show fewer steps than requested.
 *   RST [1|2]           8 DN pulses (both PIs if omitted)     -> OK
 *   EOM [ms]            count errors for ms (default 100)     -> OK <errors> <total>
 *   LVL                 instantaneous EOM_ERR level           -> OK <0|1>
 *   DAC <A|B> <mV>      set a DAC output (0..3300 mV)         -> OK <code>
 *   HELP                command list
 */
#include "stm32l1xx_hal.h"
#include "pi.h"
#include "eom.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define VDDA_MV             3300u
#define DAC_A_DEFAULT_MV    1600u
#define DAC_B_DEFAULT_MV    200u
#define CHIP_STARTUP_MS     10u      /* let the chip's clock start before resetting PIs */
#define EOM_DEFAULT_MS      100u
#define EOM_MAX_MS          60000u
#define PI_MAX_PULSES       PI_STEPS /* per command; more is pointless with 8 positions */
#define LINE_MAX            64

DAC_HandleTypeDef  hdac;
UART_HandleTypeDef huart2;

static uint32_t dac_mv[2];

static void clock_init(void);
static void dac_init(void);
static uint32_t dac_set_mv(int ch, uint32_t mv);
static void uart_init(void);
static void reply(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void handle_line(char *line);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    clock_init();

    pi_init();                       /* outputs low before anything else */
    HAL_Delay(CHIP_STARTUP_MS);
    pi_reset(1);
    pi_reset(2);

    dac_init();
    dac_set_mv(0, DAC_A_DEFAULT_MV);
    dac_set_mv(1, DAC_B_DEFAULT_MV);

    eom_init();
    uart_init();
    reply("READY");

    char line[LINE_MAX];
    size_t len = 0;

    while (1) {
        if (!(USART2->SR & USART_SR_RXNE)) continue;
        char c = (char)(USART2->DR & 0xFF);       /* reading DR also clears ORE */

        if (c == '\r' || c == '\n') {
            if (len) {
                line[len] = '\0';
                handle_line(line);
                len = 0;
            }
        } else if (len < LINE_MAX - 1) {
            line[len++] = c;
        } else {
            len = 0;                              /* overlong line: drop it */
            reply("ERR LINE_TOO_LONG");
        }
    }
}

/* ------------------------------------------------------------- commands */

static int parse_pi(const char *s)
{
    if (!s) return 0;
    int v = atoi(s);
    return (v >= 1 && v <= PI_COUNT) ? v : 0;
}

static void handle_line(char *line)
{
    for (char *p = line; *p; p++) *p = (char)toupper((unsigned char)*p);

    char *cmd = strtok(line, " \t");
    char *a1  = strtok(NULL, " \t");
    char *a2  = strtok(NULL, " \t");
    if (!cmd) return;

    if (!strcmp(cmd, "ID")) {
        reply("OK CHIPS-CTRL 1.0");

    } else if (!strcmp(cmd, "STATUS")) {
        reply("OK PI1=%ld PI2=%ld DACA=%lu DACB=%lu ERR=%d",
              (long)pi_position(1), (long)pi_position(2),
              (unsigned long)dac_mv[0], (unsigned long)dac_mv[1], eom_err_level());

    } else if (!strcmp(cmd, "UP") || !strcmp(cmd, "DN")) {
        int pi = parse_pi(a1);
        uint32_t n = a2 ? strtoul(a2, NULL, 10) : 1u;
        if (!pi)                         { reply("ERR BAD_PI"); return; }
        if (n == 0 || n > PI_MAX_PULSES) { reply("ERR BAD_COUNT"); return; }
        if (cmd[0] == 'U') pi_up(pi, n); else pi_down(pi, n);
        reply("OK PI%d=%ld", pi, (long)pi_position(pi));

    } else if (!strcmp(cmd, "RST")) {
        if (a1) {
            int pi = parse_pi(a1);
            if (!pi) { reply("ERR BAD_PI"); return; }
            pi_reset(pi);
        } else {
            pi_reset(1);
            pi_reset(2);
        }
        reply("OK");

    } else if (!strcmp(cmd, "EOM")) {
        uint32_t ms = a1 ? strtoul(a1, NULL, 10) : EOM_DEFAULT_MS;
        uint32_t errors, total;
        if (ms == 0 || ms > EOM_MAX_MS) { reply("ERR BAD_MS"); return; }
        eom_status_t st = eom_measure(ms, &errors, &total);
        if (st == EOM_NO_CLOCK)      reply("ERR NO_CLOCK");
        else if (st == EOM_OVERFLOW) reply("ERR OVERFLOW");
        else reply("OK %lu %lu", (unsigned long)errors, (unsigned long)total);

    } else if (!strcmp(cmd, "LVL")) {
        reply("OK %d", eom_err_level());

    } else if (!strcmp(cmd, "DAC")) {
        int ch = (a1 && a1[0] == 'A' && !a1[1]) ? 0 : (a1 && a1[0] == 'B' && !a1[1]) ? 1 : -1;
        if (ch < 0 || !a2) { reply("ERR USAGE: DAC <A|B> <mV>"); return; }
        uint32_t mv = strtoul(a2, NULL, 10);
        if (mv > VDDA_MV)  { reply("ERR BAD_MV"); return; }
        reply("OK %lu", (unsigned long)dac_set_mv(ch, mv));

    } else if (!strcmp(cmd, "HELP")) {
        reply("OK ID STATUS UP DN RST EOM LVL DAC");

    } else {
        reply("ERR UNKNOWN");
    }
}

static void reply(const char *fmt, ...)
{
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof buf - 3) n = (int)sizeof buf - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100);
}

/* ------------------------------------------------------------------ DAC */

/* VOUT = VDDA * code / 4095 (VREF+ is tied to VDDA on LQFP64).
 * Buffered output range is 0.2 V .. VDDA - 0.2 V. */
static uint32_t dac_set_mv(int ch, uint32_t mv)
{
    uint32_t code = (mv * 4095u + VDDA_MV / 2u) / VDDA_MV;
    if (code > 4095u) code = 4095u;
    HAL_DAC_SetValue(&hdac, ch ? DAC_CHANNEL_2 : DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);
    dac_mv[ch ? 1 : 0] = mv;
    return code;
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

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
}

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

/* ----------------------------------------------------------------- UART */

static void uart_init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};

    if (h->Instance != USART2) return;
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin = GPIO_PIN_2 | GPIO_PIN_3;     /* PA2 TX, PA3 RX -> ST-LINK VCP */
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);
}

/* --------------------------------------------------------------- clocks */

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
