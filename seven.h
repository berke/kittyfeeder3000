/* seven.h
 *
 * Seven-segment display driver - header.
 *
 * (C)2011 O. Berke A. Durak
 */

#ifndef SEVEN_H_20111225

#include <inttypes.h>

/* 
 *  -E-
 * |   |
 * F   D
 * |   |
 *  -G-
 * |   |
 * A   C
 * |   |
 *  -B-  (P)
 *
 */

#define SS_A   1
#define SS_B   2
#define SS_C   4
#define SS_P   8
#define SS_D  16
#define SS_E  32
#define SS_F  64
#define SS_G 128

struct seven_state
{
        uint8_t pattern[4];
        uint16_t  word;    /* Word to display */
};

void seven_init(volatile struct seven_state *q);
void seven_process(volatile struct seven_state *q);
void seven_set(volatile struct seven_state *q, uint16_t w);
uint32_t seven_encode(uint16_t x);
void seven_set_pattern(volatile struct seven_state *q, uint32_t pattern);
uint16_t seven_get(volatile struct seven_state *q);
uint16_t seven_to_bcd(uint16_t x);

#endif
