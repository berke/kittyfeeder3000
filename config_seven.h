/* seven.c
 *
 * Seven-segment display driver - configuration.
 *
 * (C)2011 O. Berke A. Durak
 */

#ifndef CONFIG_SEVEN_H_20111225
#define CONFIG_SEVEN_H_20111225

#include <avr/io.h>

#define SEVEN_SHR_PORT PORTD
#define SEVEN_SHR_DDR DDRD
#define SEVEN_SHR_PIN PIND
#define SEVEN_SHR_DAT 5
#define SEVEN_SHR_CLK 6
#define SEVEN_SHR_RST 7

#define SEVEN_DGT_PORT PORTC
#define SEVEN_DGT_DDR DDRC
#define SEVEN_DGT_PIN PINC
#define SEVEN_DGT_3 8
#define SEVEN_DGT_2 4
#define SEVEN_DGT_1 2
#define SEVEN_DGT_0 1

#endif
