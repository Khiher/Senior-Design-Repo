/**
 * EOM error counter, clocked by the chip's 100 MHz / 16 = 6.25 MHz clock.
 *
 *   EOM_ERR -> PA1  -> Arduino A1  (TIM2_CH2, gate)
 *   EOM_CLK -> PA0  -> Arduino A0  (TIM2_ETR, counts error cycles)
 *           -> PB6  -> Arduino D10 (TIM4_CH1, counts all cycles)
 *   EOM_CLK must be wired to BOTH A0 and D10.
 *
 * TIM2 runs in external clock mode 2 (ETR = EOM_CLK) combined with gated slave mode
 * (gate = EOM_ERR): it counts clock edges while the error line is high, i.e. the
 * number of error cycles. TIM4 counts every clock edge in the same window.
 *
 * Edges are counted on the FALLING edge of EOM_CLK, the middle of each error bit,
 * assuming the chip changes EOM_ERR on the rising edge. Flip EOM_CLK_FALLING in eom.c
 * if it's the other way round.
 */
#ifndef EOM_H
#define EOM_H

#include <stdint.h>

typedef enum {
    EOM_OK = 0,
    EOM_NO_CLOCK = -1,    /* no EOM_CLK edges seen during the window */
    EOM_OVERFLOW = -2,    /* clock faster than expected, a chunk wrapped 16 bits */
} eom_status_t;

void         eom_init(void);
eom_status_t eom_measure(uint32_t ms, uint32_t *errors, uint32_t *total);
int          eom_err_level(void);   /* instantaneous EOM_ERR pin level, 0 or 1 */

#endif
