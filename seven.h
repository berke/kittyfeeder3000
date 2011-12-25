/* seven.h
 *
 * Seven-segment display driver - header.
 *
 * (C)2011 O. Berke A. Durak
 */

#ifndef SEVEN_H_20111225

#include <inttypes.h>

struct seven_state
{
        uint16_t  word;    /* Word to display */
};

void seven_init(volatile struct seven_state *q);
void seven_process(volatile struct seven_state *q);
void seven_set(volatile struct seven_state *q, uint16_t x);
uint16_t seven_get(volatile struct seven_state *q);
uint16_t seven_to_bcd(uint16_t x);

#endif
