/* KittyFeeder 3000 */

#define FAST_DEBUG 0

#define F_CPU 1000000

const char copyright[] =
"Copyright (C)2010 Oguz Berke Antoine DURAK.  All rights reserved.";

#include <string.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

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

struct clock_state {
	uint32_t t_s;
	uint32_t t_us;
	uint32_t t_s_last;
	uint16_t encoded;
	uint32_t ticks;
};

void status_led_set(void)   { LED1_PORT |=  _BV(LED1_BIT); }
void status_led_clear(void) { LED1_PORT &= ~_BV(LED1_BIT); }
void status_led_init(void)   { LED1_DDR |= _BV(LED1_BIT); status_led_clear(); }

uint16_t status = 0;

volatile struct seven_state seven;
volatile struct clock_state clock;

void timer_init(void)
{
        /* 58kHz generator */
#if 0
        TCCR1A = _BV(COM1B1) | _BV(WGM10);
        TCCR1B = _BV(WGM13) | _BV(CS10) | _BV(CS12);
        OCR1A = 2;
        OCR1B = 1;
        TIMSK = _BV(TOIE1);
#endif
	if (false)
        	TCCR0 = _BV(CS00);
	else
		TCCR0 = _BV(CS02) | _BV(CS00);

	TCNT0 = 0;
        TIMSK = _BV(TOIE0);
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

#define ADC_BITS 10

void adc_init(void)
{
        ADMUX = _BV(REFS0) | _BV(MUX0) | _BV(MUX2);
        ADCSRA = _BV(ADEN);
}

uint16_t adc_get(void)
{
        ADCSRA |= _BV(ADSC);
        do { } while(ADCSRA & _BV(ADSC));
        return ADC;
}

uint16_t adc_get_choice(uint16_t n)
{
	uint16_t pot;

	pot = adc_get();
	pot = (pot * n) >> ADC_BITS;
	if (pot >= n) pot = n - 1;
	return pot;
}

#define CLOCK_PERIOD_US 262144

void clock_init(volatile struct clock_state *ck,
		uint8_t hour, uint8_t min, uint8_t sec)
{
	ck->t_s = sec + 60 * (min + 60 * hour);
	ck->t_s_last = 0;
	ck->t_us = 0;
	ck->ticks = 0;
}

void clock_set_h(volatile struct clock_state *ck, uint8_t h)
{
	uint8_t m;
	uint16_t minutes;

	cli();
	minutes = ck->t_s / 60;
	m = minutes % 60;
	ck->t_s = 60 * (h * 60 + m);
	sei();
}

uint16_t clock_encode(volatile struct clock_state *ck)
{
	uint8_t h, m;
	uint16_t minutes;

	if (ck->t_s == ck->t_s_last)
		return ck->encoded;

	ck->t_s_last = ck->t_s;

	minutes = ck->t_s / 60;
	m = minutes % 60;
	h = (minutes / 60) % 24;

	ck->encoded = seven_to_bcd((h * 100) + m);

	return ck->encoded;
}

void clock_tick(volatile struct clock_state *ck)
{
	ck->ticks ++;
	ck->t_us += CLOCK_PERIOD_US;
	if (ck->t_us >= 1000000) {
		ck->t_us -= 1000000;
		ck->t_s ++;
	}
}

uint32_t clock_get_ticks(volatile struct clock_state *ck)
{
	uint32_t t;

	cli();
	t = ck->ticks;
	sei();
	return t;
}

/* 32µs timer */
ISR(SIG_OVERFLOW0)
{
	clock_tick(&clock);
}

#define BUTTON_PORT PORTC
#define BUTTON_DDR DDRC
#define BUTTON_PIN PINC
#define BUTTON_BIT 4
#define BUTTON_LO_THRESHOLD 2
#define BUTTON_HI_THRESHOLD 6
#define BUTTON_LIMIT 8

struct button_state {
	bool state;
	uint8_t count;
};

void button_init(struct button_state *q)
{
	BUTTON_DDR &= ~_BV(BUTTON_BIT);
	BUTTON_PORT |= _BV(BUTTON_BIT);
	q->count = 0;
}

bool button_tick(struct button_state *q)
{
	if (BUTTON_PIN & _BV(BUTTON_BIT)) {
		if (q->count)
			q->count --;
	} else {
		if (q->count < BUTTON_LIMIT)
			q->count ++;
	}

	if (q->state) {
		if (q->count < BUTTON_LO_THRESHOLD)
			q->state = false;
	} else {
		if (q->count > BUTTON_HI_THRESHOLD)
			q->state = true;
	}

	return q->state;
}

static struct button_state button;

#define A SS_A
#define B SS_B
#define C SS_C
#define P SS_P
#define D SS_D
#define E SS_E
#define F SS_F
#define G SS_G

#define PAT(a,b,c,d) ((((uint32_t) a) << 24) | (((uint32_t) b) << 16) \
		| (((uint32_t) c) << 8) | ((uint32_t) d))

#define MAIN_BANNER_1_TIME 2
#define MAIN_BANNER_1_PATTERN PAT(G|A|B, E|D|G|A|C|B, F|G|A|B, 0)
#define MAIN_BANNER_2_TIME 2
#define MAIN_BANNER_2_PATTERN PAT(E|F|G|A, E|D|F|G|A|B, E|D|F|G|A|B, D|G|A|C|B)
#define MAIN_BANNER_3_TIME 2

#define MAIN_MIN_PATTERN PAT(E|F|D|A|C, A, G|A|C|P, 0)
#define MAIN_HOUR_PATTERN PAT(F|D|G|A|C, G|A|P, 0, 0)
#define MAIN_FEED_PATTERN PAT(E|F|G|A, E|F|D|G|A|B, E|F|D|G|A|B, D|G|A|C|B)
#define MAIN_CANCEL_PATTERN PAT(E|F|A|B, E|D|G|A|C|B, G|A|C, G|A|B|P)

enum {
	MAIN_SELM_HOURS,
	MAIN_SELM_MINUTES,
	MAIN_SELM_FEED,
	MAIN_SELM_CANCEL,
	MAIN_SELM_COUNT
};

enum {
	MAIN_BANNER_1,
	MAIN_BANNER_2,
	MAIN_BANNER_3,
	MAIN_DISP_TIME,
	MAIN_SELM,
	MAIN_SET_H,
	MAIN_SET_M,
	MAIN_SET_FEED,
};

int main(void)
{
	bool last_prs, prs = false;
	bool but_prs;
	uint8_t state = MAIN_BANNER_1;
	uint8_t select_state;
	uint32_t t, t_start = 0;
	uint8_t h, m;
	
        timer_init();
        seven_init(&seven);
        serial_init();
        adc_init();
	clock_init(&clock, 8, 39, 0);
	button_init(&button);
        sei();

        for(;;) {
		t = clock_get_ticks(&clock);
		last_prs = prs;
		prs = button_tick(&button);
		but_prs = !last_prs && prs;

		switch (state) {
			case MAIN_BANNER_1:
				seven_set_pattern(&seven,
						MAIN_BANNER_1_PATTERN);
				if (t > t_start + MAIN_BANNER_1_TIME) {
					t_start = t;
					state = MAIN_BANNER_2;
				}
				break;
			case MAIN_BANNER_2:
				seven_set_pattern(&seven,
						MAIN_BANNER_2_PATTERN);
				if (t > t_start + MAIN_BANNER_2_TIME) {
					t_start = t;
					state = MAIN_BANNER_3;
				}
				break;
			case MAIN_BANNER_3:
				seven_set(&seven, 0x3000);
				if (t > t_start + MAIN_BANNER_3_TIME) {
					state = MAIN_DISP_TIME;
				}
				break;
			case MAIN_DISP_TIME:
				seven_set(&seven, clock_encode(&clock));
				if (!last_prs && prs) {
					select_state = MAIN_SELM_HOURS;
					state = MAIN_SELM;
				}
				break;

			case MAIN_SELM:
				select_state = adc_get_choice(MAIN_SELM_COUNT);
				switch (select_state) {
					case MAIN_SELM_MINUTES:
						seven_set_pattern(&seven,
							MAIN_MIN_PATTERN);
						if (but_prs)
							state = MAIN_SET_M;
						break;

					case MAIN_SELM_HOURS:
						seven_set_pattern(&seven,
							MAIN_HOUR_PATTERN);
						if (but_prs)
							state = MAIN_SET_H;
						break;

					case MAIN_SELM_FEED:
						seven_set_pattern(&seven,
							MAIN_FEED_PATTERN);
						if (but_prs)
							state = MAIN_SET_FEED;
						break;

					default:
					case MAIN_SELM_CANCEL:
						seven_set_pattern(&seven,
							MAIN_CANCEL_PATTERN);
						if (but_prs)
							state = MAIN_DISP_TIME;
						break;
				}
				break;

			case MAIN_SET_H:
				h = adc_get_choice(24);
				seven_set(&seven, seven_to_bcd(h));
				if (but_prs) {
					clock_set_h(&clock, h);
					state = MAIN_DISP_TIME;
				}
				break;

			default:
				break;
		}

                seven_process(&seven);
        }
}

/* vim:set ts=8 sw=8 noexpandtab tw=80 cc=80: */
