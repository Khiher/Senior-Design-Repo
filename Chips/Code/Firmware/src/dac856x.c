#include "dac856x.h"

#define DAC_SPI_TIMEOUT_MS 10

static inline void sync_low(dac856x_t *dac)  { HAL_GPIO_WritePin(dac->sync_port, dac->sync_pin, GPIO_PIN_RESET); }
static inline void sync_high(dac856x_t *dac) { HAL_GPIO_WritePin(dac->sync_port, dac->sync_pin, GPIO_PIN_SET); }

HAL_StatusTypeDef dac_write_frame(dac856x_t *dac, uint8_t cmd, uint8_t addr, uint16_t data)
{
    uint8_t frame[3] = {
        (uint8_t)((cmd & 0x38u) | (addr & 0x07u)),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFFu),
    };

    sync_low(dac);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(dac->hspi, frame, sizeof frame, DAC_SPI_TIMEOUT_MS);
    /* HAL_SPI_Transmit returns once the last byte has left the shifter (BSY cleared),
     * so the 24th falling edge has occurred. SYNC high time must be >= 80 ns (t4). */
    sync_high(dac);
    return st;
}

static uint16_t align_code(const dac856x_t *dac, uint16_t code)
{
    uint16_t max = (uint16_t)((1u << dac->resolution) - 1u);
    if (code > max) code = max;
    return (uint16_t)(code << (16u - dac->resolution));
}

HAL_StatusTypeDef dac_set(dac856x_t *dac, dac_addr_t ch, uint16_t code)
{
    return dac_write_frame(dac, DAC_CMD_WRITE_UPDATE_N, ch, align_code(dac, code));
}

HAL_StatusTypeDef dac_write_buffered(dac856x_t *dac, dac_addr_t ch, uint16_t code)
{
    return dac_write_frame(dac, DAC_CMD_WRITE_INPUT, ch, align_code(dac, code));
}

HAL_StatusTypeDef dac_load(dac856x_t *dac, dac_addr_t ch)
{
    return dac_write_frame(dac, DAC_CMD_UPDATE_DAC, ch, 0);
}

HAL_StatusTypeDef dac_power(dac856x_t *dac, uint8_t sel, dac_power_t mode)
{
    return dac_write_frame(dac, DAC_CMD_POWER, 0, (uint16_t)(((mode & 0x3u) << 4) | (sel & 0x3u)));
}

HAL_StatusTypeDef dac_internal_ref(dac856x_t *dac, int enable)
{
    /* Also resets both channels' gain: 2 when enabled, 1 when disabled (Table 9). */
    return dac_write_frame(dac, DAC_CMD_INTERNAL_REF, 0, enable ? 1u : 0u);
}

HAL_StatusTypeDef dac_set_gain(dac856x_t *dac, int gain_a, int gain_b)
{
    /* Table 7: bit = 1 -> gain 1, bit = 0 -> gain 2 */
    uint16_t bits = (uint16_t)((gain_b == 1 ? DAC_SEL_B : 0u) | (gain_a == 1 ? DAC_SEL_A : 0u));
    return dac_write_frame(dac, DAC_CMD_WRITE_INPUT, DAC_GAIN, bits);
}

HAL_StatusTypeDef dac_set_ldac_mode(dac856x_t *dac, uint8_t sync_sel)
{
    return dac_write_frame(dac, DAC_CMD_LDAC, 0, sync_sel & 0x3u);
}

HAL_StatusTypeDef dac_reset(dac856x_t *dac, int full)
{
    return dac_write_frame(dac, DAC_CMD_RESET, 0, full ? 1u : 0u);
}

void dac_pulse_ldac(dac856x_t *dac)
{
    if (!dac->ldac_port) return;
    HAL_GPIO_WritePin(dac->ldac_port, dac->ldac_pin, GPIO_PIN_RESET); /* falling edge latches */
    __NOP(); __NOP(); __NOP(); __NOP();                               /* t12 >= 10 ns         */
    HAL_GPIO_WritePin(dac->ldac_port, dac->ldac_pin, GPIO_PIN_SET);   /* must be high before next frame */
}

void dac_pulse_clr(dac856x_t *dac)
{
    if (!dac->clr_port) return;
    HAL_GPIO_WritePin(dac->clr_port, dac->clr_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 50; i++) { }                         /* t13 >= 80 ns */
    HAL_GPIO_WritePin(dac->clr_port, dac->clr_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef dac_init(dac856x_t *dac, int use_internal_ref)
{
    HAL_StatusTypeDef st;

    sync_high(dac);
    if (dac->clr_port)  HAL_GPIO_WritePin(dac->clr_port, dac->clr_pin, GPIO_PIN_SET);
    if (dac->ldac_port) HAL_GPIO_WritePin(dac->ldac_port, dac->ldac_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    if ((st = dac_reset(dac, 1)) != HAL_OK) return st;                 /* all registers to POR */
    if ((st = dac_power(dac, DAC_SEL_AB, DAC_PWR_UP)) != HAL_OK) return st;
    /* Both channels update on the 24th SCLK edge regardless of the LDAC pin. */
    if ((st = dac_set_ldac_mode(dac, DAC_SEL_AB)) != HAL_OK) return st;
    if ((st = dac_internal_ref(dac, use_internal_ref)) != HAL_OK) return st;
    return HAL_OK;
}
