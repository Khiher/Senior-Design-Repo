/**
 * Driver for TI DAC7562/7563 (12-bit), DAC8162/8163 (14-bit), DAC8562/8563 (16-bit)
 * dual-channel SPI DACs. Reference: TI SLAS719E.
 *
 * Frame format (24 bits, MSB first, Table 14-17):
 *   [23:22] don't care
 *   [21:19] command C2..C0
 *   [18:16] address A2..A0
 *   [15:0]  data (DAC code left-aligned for 14/12-bit parts)
 *
 * Data is latched on SCLK falling edges -> SPI mode 1 (CPOL=0, CPHA=1).
 */
#ifndef DAC856X_H
#define DAC856X_H

#include "stm32l1xx_hal.h"
#include <stdint.h>

/* Commands, Table 12 (already shifted into bits [21:19] of the frame -> [5:3] of byte 0) */
#define DAC_CMD_WRITE_INPUT        (0x0u << 3) /* write input register n                 */
#define DAC_CMD_UPDATE_DAC         (0x1u << 3) /* software LDAC, update DAC register n   */
#define DAC_CMD_WRITE_UPDATE_ALL   (0x2u << 3) /* write input n, update all DAC regs     */
#define DAC_CMD_WRITE_UPDATE_N     (0x3u << 3) /* write input n, update DAC register n   */
#define DAC_CMD_POWER              (0x4u << 3) /* power up / power down                  */
#define DAC_CMD_RESET              (0x5u << 3) /* software reset                         */
#define DAC_CMD_LDAC               (0x6u << 3) /* set LDAC registers                     */
#define DAC_CMD_INTERNAL_REF       (0x7u << 3) /* enable/disable internal reference      */

/* Addresses, Table 13 */
typedef enum {
    DAC_CH_A    = 0x0,
    DAC_CH_B    = 0x1,
    DAC_GAIN    = 0x2, /* only with DAC_CMD_WRITE_INPUT */
    DAC_CH_BOTH = 0x7,
} dac_addr_t;

/* Power modes, Table 5 (DB5:DB4) */
typedef enum {
    DAC_PWR_UP        = 0x0,
    DAC_PWR_DOWN_1K   = 0x1,
    DAC_PWR_DOWN_100K = 0x2,
    DAC_PWR_DOWN_HIZ  = 0x3,
} dac_power_t;

/* Channel select bits for power/LDAC/gain commands (DB1 = B, DB0 = A) */
#define DAC_SEL_A   0x1u
#define DAC_SEL_B   0x2u
#define DAC_SEL_AB  0x3u

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *sync_port;  uint16_t sync_pin;   /* SYNC (active-low frame select) */
    GPIO_TypeDef *ldac_port;  uint16_t ldac_pin;   /* optional, NULL if tied to GND  */
    GPIO_TypeDef *clr_port;   uint16_t clr_pin;    /* optional, NULL if tied to AVDD */
    uint8_t resolution;                            /* 12, 14 or 16                   */
} dac856x_t;

HAL_StatusTypeDef dac_write_frame(dac856x_t *dac, uint8_t cmd, uint8_t addr, uint16_t data);

/* Reset, power up both channels, sync (LDAC-less) updates, optional internal 2.5 V ref (gain=2). */
HAL_StatusTypeDef dac_init(dac856x_t *dac, int use_internal_ref);

/* Write a code (0 .. 2^resolution-1) and update the output immediately. */
HAL_StatusTypeDef dac_set(dac856x_t *dac, dac_addr_t ch, uint16_t code);

/* Write input register only; outputs change on dac_load() or an LDAC pulse. */
HAL_StatusTypeDef dac_write_buffered(dac856x_t *dac, dac_addr_t ch, uint16_t code);
HAL_StatusTypeDef dac_load(dac856x_t *dac, dac_addr_t ch);
void dac_pulse_ldac(dac856x_t *dac);

HAL_StatusTypeDef dac_power(dac856x_t *dac, uint8_t sel, dac_power_t mode);
HAL_StatusTypeDef dac_internal_ref(dac856x_t *dac, int enable);
HAL_StatusTypeDef dac_set_gain(dac856x_t *dac, int gain_a, int gain_b); /* 1 or 2 */
HAL_StatusTypeDef dac_set_ldac_mode(dac856x_t *dac, uint8_t sync_sel);   /* 1 = sync, ignores LDAC pin */
HAL_StatusTypeDef dac_reset(dac856x_t *dac, int full);
void dac_pulse_clr(dac856x_t *dac);

#endif /* DAC856X_H */
