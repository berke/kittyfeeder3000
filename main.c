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

static uint16_t to_bcd(uint16_t x)
{
  uint16_t r = 0;

  if(x >= 10000) return 0x9999;
  while(x >= 1000) { r += 0x1000; x -=   1000; }
  while(x >=  100) { r += 0x0100; x -=    100; }
  while(x >=   10) { r += 0x0010; x -=     10; }
  r += x;
  return r;
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
  _delay_us(50);
}

static void seven_init(volatile seven_state *q)
{
  DDRD  |=  7 << 5;
  DDRC  |= 15;

  q->word    = 0x1234;
}

static void seven_set(uint16_t x)
{
  seven.word = x;
}

static uint16_t seven_get(void)
{
  return seven.word;
}

static void seven_process(volatile seven_state *q)
{
  uint16_t w;
  uint8_t p0, p1, p2, p3;
  uint8_t i;

  w = q->word;

  p0 = seven_table[w & 15]; w >>= 4;
  p1 = seven_table[w & 15]; w >>= 4;
  p2 = seven_table[w & 15]; w >>= 4;
  p3 = seven_table[w & 15];

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

void serial_init(void)
{
  UBRRH = 0;
  UBRRL = 12;
  UCSRB |= _BV(RXEN) | _BV(TXEN);
  UCSRC = _BV(URSEL) | _BV(UCSZ0) | _BV(UCSZ1);
}

uint8_t char_of_nibble(uint8_t x)
{
  return x < 10 ? '0' + x : 'a' + x - 10;
}

void serial_send(uint8_t c)
{
  while(!(UCSRA & _BV(UDRE)));
  UDR = c;
}

void adc_init(void)
{
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX0) | _BV(MUX2);
  ADCSRA = _BV(ADEN);
}

uint16_t adc_get(void)
{
  ADCSRA |= _BV(ADSC);
  do { } while(ADCSRA & _BV(ADSC));
  return ADC;
}

#define KBD_DAT  2
#define KBD_CLK  3
#define KBD_PORT PORTD
#define KBD_PIN  PIND
#define KBD_DDR  DDRD

uint8_t kbd_count;
uint16_t kbd_state;
volatile uint16_t kbd_input;

void kbd_init(void)
{
  KBD_DDR &= ~(_BV(KBD_DAT) | _BV(KBD_CLK));
  kbd_state = 0;
  kbd_count = 0;
  kbd_input = 0;
  MCUCR |= _BV(ISC11);
  MCUCR &= ~_BV(ISC10);
  GICR |= _BV(INT1);
}

static inline void kbd_process(void)
{
  uint8_t dt;

  dt = !!(KBD_PIN & _BV(KBD_DAT));
  kbd_state <<= 1;
  kbd_state |= dt;
  kbd_count ++;
  if(kbd_count == 11)
  {
    kbd_input = kbd_state;
    kbd_state = 0;
    kbd_count = 0;
  }
}

ISR(SIG_INTERRUPT1)
{
  kbd_process();
}

/* 32µs timer */
ISR(SIG_OVERFLOW1)
{
}

int main(void)
{
  uint32_t q_total = 0, q;
  uint16_t display_word = 0;
  uint16_t c = 0;
  uint16_t got_key;
  uint16_t goal = 3000;
  uint8_t display_mode = 0;
  uint16_t display_count = 0;

  timer_init();
  seven_init(&seven);
  serial_init();
  kbd_init();
  adc_init();
  sei();

  for(;;) {
    seven_process(&seven);

    //cli();
    got_key = kbd_input;
    kbd_input = 0;
    //sei();
    switch(got_key)
    {
      case 0x2b9:
        goal += 10;
        display_mode = 1;
        display_count = 100;
        break;
      case 0x13b:
        goal -= 10;
        display_mode = 1;
        display_count = 100;
        break;
      default:
        display_mode = 2;
        display_word = got_key;
        display_count = 200;
        serial_send(char_of_nibble(got_key & 0x00f));
        serial_send(char_of_nibble((got_key >> 4) & 0x00f));
        serial_send(char_of_nibble((got_key >> 8) & 0x00f));
        break;
      case 0:
        break;
    }

    // seven.word = bcd_inc(seven.word);
    if(display_mode)
    {
      PORTB |= 2;
      switch(display_mode)
      {
        case 1:
          seven_set(to_bcd(goal));
          break;
        case 2:
          seven_set(display_word);
          break;
      }
      display_count --;
      if(!display_count) display_mode = 0;
    }
    else
    {
      q  = adc_get();
      q_total += q;
      c ++;
      if(c == 32)
      {
        c = 0;
        q_total >>= 5;
        q_total *= 9999;
        q_total >>= 10;
        seven_set(to_bcd(q_total));
        q_total = 0;
      }
      PORTB &= ~2;
    }

    retard();
  }
}
