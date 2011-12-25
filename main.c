/* KittyFeeder 3000 */

#define FAST_DEBUG 0

#define F_CPU 1000000

const char copyright[] =
"Copyright (C)2010 Oguz Berke Antoine DURAK.  All rights reserved.";

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "seven.h"

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

volatile struct seven_state seven;

typedef uint8_t bool;

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
                                        seven_set(&seven, seven_to_bcd(goal));
                                        break;
                                case 2:
                                        seven_set(&seven, display_word);
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
                                seven_set(&seven, seven_to_bcd(q_total));
                                q_total = 0;
                        }
                        PORTB &= ~2;
                }

                //retard();
        }
}

/* vim:set ts=8 sw=8 noexpandtab tw=80 cc=80: */
