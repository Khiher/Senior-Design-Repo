/**
 * Phase-interpolator step control: two PIs, each with an UP and a DN pulse input.
 *
 *   PI1_UP = PB10 -> Arduino D6     PI1_DN = PA8 -> Arduino D7
 *   PI2_UP = PA9  -> Arduino D8     PI2_DN = PC7 -> Arduino D9
 *
 * The chip steps once per pulse (edge-triggered, synchronized to its 100 MHz clock
 * internally), so the MCU only has to produce clean pulses wider than the chip's
 * synchronizer needs (a few 10 ns periods). Widths are set in pi.c.
 */
#ifndef PI_H
#define PI_H

#include <stdint.h>

#define PI_COUNT          2
#define PI_STEPS          8     /* positions per PI: 0 .. PI_STEPS-1 */
#define PI_POS_MAX        (PI_STEPS - 1)
#define PI_RESET_PULSES   8u    /* DN pulses at startup / on reset (spec: at least 8) */

/* Positions are tracked by counting pulses and clamped to 0..PI_POS_MAX: pi_up/pi_down
 * only send the pulses that stay inside the range, so the chip never gets a pulse past
 * either end. The tracking is only valid after a reset. */
void    pi_init(void);                       /* GPIOs low, cycle counter on */
int32_t pi_up(int pi, uint32_t n);           /* pi = 1 or 2; returns pulses actually sent */
int32_t pi_down(int pi, uint32_t n);
void    pi_reset(int pi);                    /* PI_RESET_PULSES DN pulses, position = 0 */
int32_t pi_position(int pi);                 /* 0 .. PI_POS_MAX */

#endif
