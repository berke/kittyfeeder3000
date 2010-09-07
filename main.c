/* Basic blinker */

#define FAST_DEBUG 0

#define F_CPU 1000000

const char copyright[] = "Copyright (C)2010 Oguz Berke Antoine DURAK.  All rights reserved.";

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "types.h"

#define CHECK 1
#if CHECK
#define CK(x,y,z) cli(); if(!(y)) { sei(); alarm(x); z; } else { sei(); }
#else
#define CK(x,y,z)
#endif

#define LED1_DDR DDRC
#define LED1_PORT PORTC
#define LED1_BIT 5

#define SEGSEL0_DDR DDRD
#define SEGSEL1_DDR DDRD
#define SEGSEL2_DDR DDRD

#define SEGSEL0_PORT DDRD
#define SEGSEL1_PORT DDRD
#define SEGSEL2_PORT DDRD

void status_led_set(void)   { LED1_PORT |=  _BV(LED1_BIT); }
void status_led_clear(void) { LED1_PORT &= ~_BV(LED1_BIT); }
void status_led_init(void)   { LED1_DDR |= _BV(LED1_BIT); status_led_clear(); }

uint16_t status = 0;

typedef struct seven_state_s
{
  uint16_t  word;    /* Word to display */
} seven_state;

/* 0 - normal display
 * 1 to 32 : test mode, show segment i */

#define A   1
#define B   2
#define C   4
#define P   8
#define D  16
#define E  32
#define F  64
#define G 128

static const uint8_t seven_table[] =
{
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

  /* 0 */ A+B+C+D+E+F    ,
  /* 1 */     C+D        ,
  /* 2 */ A+B  +D+E  +G  ,
  /* 3 */   B+C+D+E  +G  ,
  /* 4 */     C+D  +F+G  ,
  /* 5 */   B+C  +E+F+G  ,
  /* 6 */ A+B+C  +E+F+G  ,
  /* 7 */     C+D+E      ,
  /* 8 */ A+B+C+D+E+F+G  ,
  /* 9 */   B+C+D+E+F+G  ,
  /* A */ A  +C+D+E+F+G  ,
  /* B */ A+B+C    +F+G  ,
  /* C */ A+B    +E+F    ,
  /* D */ A+B+C+D    +G  ,
  /* E */ A+B    +E+F+G  ,
  /* F */ A      +E+F+G
};

volatile seven_state seven;

#define SEG7_PORT PORTD
#define SEG7_DAT 5
#define SEG7_CLK 6
#define SEG7_RST 7

typedef uint8_t bool;

static inline void retard(void)
{
  asm volatile(
      "nop"
    );
}

static uint16_t bcd_inc(uint16_t x)
{
  if((x & 0x000f) < 0x0009) return x + 0x0001;
  x &= 0xfff0;
  if((x & 0x00f0) < 0x0090) return x + 0x0010;
  x &= 0xff00;
  if((x & 0x0f00) < 0x0900) return x + 0x0100;
  x &= 0xf000;
  if((x & 0xf000) < 0x9000) return x + 0x1000;
  return 0;
}

static void multi_delay(uint8_t c)
{
  _delay_us(125);
}

static void seven_set(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
  uint8_t i;

  PORTC |= 15;

  SEG7_PORT &= ~(_BV(SEG7_RST) | _BV(SEG7_CLK));
  retard();
  SEG7_PORT |= _BV(SEG7_RST);
  retard();

  SEG7_PORT |= _BV(SEG7_DAT);
  retard();
  SEG7_PORT |= _BV(SEG7_CLK);
  retard();

  SEG7_PORT &= ~_BV(SEG7_DAT);
  SEG7_PORT &= ~_BV(SEG7_CLK);

  for(i = 0; i < 8; i ++)
  {
    if(p3 & 1) PORTC &= ~8; multi_delay(1); PORTC |= 8;
    if(p2 & 1) PORTC &= ~4; multi_delay(1); PORTC |= 4;
    if(p1 & 1) PORTC &= ~2; multi_delay(1); PORTC |= 2;
    if(p0 & 1) PORTC &= ~1; multi_delay(1); PORTC |= 1;

    p3 >>= 1;
    p2 >>= 1;
    p1 >>= 1;
    p0 >>= 1;

    SEG7_PORT |= _BV(SEG7_CLK);
    retard();
    SEG7_PORT &= ~_BV(SEG7_CLK);
  }
}

static void seven_init(volatile seven_state *q)
{
  DDRD  |=  7 << 5;
  DDRC  |= 15;

  q->word    = 0x1234;
}

static void seven_process(volatile seven_state *q)
{
  uint16_t w;
  uint8_t p0, p1, p2, p3;

  w = q->word;

  p0 = seven_table[w & 15]; w >>= 4;
  p1 = seven_table[w & 15]; w >>= 4;
  p2 = seven_table[w & 15]; w >>= 4;
  p3 = seven_table[w & 15];
  seven_set(p0, p1, p2, p3);
}

/* 32µs timer */
ISR(SIG_OVERFLOW1)
{
  seven.word = bcd_inc(seven.word);
  PORTB ^= 2;
}

void timer_init(void)
{
  /* 58kHz generator */
  TCCR1A = _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(CS10) | _BV(CS12);
  OCR1A = 200;
  OCR1B = 100;
  TIMSK = _BV(TOIE1);

  DDRB |= 2;
}

int main(void)
{
  timer_init();
  seven_init(&seven);
  sei();

  for(;;) {
    //set_sleep_mode(SLEEP_MODE_IDLE);
    //sleep_mode();
    seven_process(&seven);

    retard();
  }
}
