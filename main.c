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
struct program_state program;
struct feeder_state feeder;

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

#define PWM_PERIOD 10000
#define PWM_ON 1250
#define PWM_OFF 500

void pwm_init(void)
{
	ICR1 = PWM_PERIOD;
	OCR1A = PWM_OFF;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
	//TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(CS10);
	DDRB |= 2;
}

void pwm_control(bool state)
{
	OCR1A = state ? PWM_ON : PWM_OFF;
}

enum feed_state {
	FEED_IDLE,
	FEED_OPENING,
	FEED_SHAKING,
	FEED_PAUSING,
	FEED_CLOSING
};

enum feed_event {
	FEED_TICK,
	FEED_KITTY
};

#define FEED_OPEN_TIME 5
#define FEED_SHAKE_TIME 10
#define FEED_PAUSE_TIME 3
#define FEED_CLOSE_TIME 5

struct feeder_state {
	uint8_t state;
	uint8_t timer;
};

#define SHAKER_PORT PORTD
#define SHAKER_DDR DDRD
#define SHAKER_PIN PIND
#define SHAKER_BIT 2

void shaker_init(void)
{
	SHAKER_PORT &= ~_BV(SHAKER_BIT);
	SHAKER_DDR |= _BV(SHAKER_BIT);
}

void shaker_control(bool control)
{
	if (control)
		SHAKER_PORT |= _BV(SHAKER_BIT);
	else
		SHAKER_PORT &= ~_BV(SHAKER_BIT);
}

void feeder_init(struct feeder_state *q)
{
	q->state = FEED_IDLE;
	q->timer = 0;
}

void feeder_transition(struct feeder_state *q, enum feed_event e)
{
	switch (q->state) {
		default:
		case FEED_IDLE:
			switch (e) {
				case FEED_TICK:
					break;
				case FEED_KITTY:
					pwm_control(true);
					q->timer = 0;
					q->state = FEED_OPENING;
					break;
			}
			break;

		case FEED_OPENING:
			q->timer ++;
			if (q->timer == FEED_OPEN_TIME) {
				shaker_control(true);
				q->timer = 0;
				q->state = FEED_SHAKING;
			}
			break;

		case FEED_SHAKING:
			q->timer ++;
			if (q->timer == FEED_SHAKE_TIME) {
				shaker_control(false);
				q->timer = 0;
				q->state = FEED_PAUSING;
			}
			break;

		case FEED_PAUSING:
			q->timer ++;
			if (q->timer == FEED_PAUSE_TIME) {
				pwm_control(false);
				q->timer = 0;
				q->state = FEED_CLOSING;
			}
			break;

		case FEED_CLOSING:
			q->timer ++;
			if (q->timer == FEED_CLOSE_TIME) {
				q->timer = 0;
				q->state = FEED_IDLE;
			}
			break;
	}
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
	ck->t_s = 60 * ((uint32_t) h * 60 + m);
	ck->t_s_last = 0;
	sei();
}

void clock_set_m(volatile struct clock_state *ck, uint8_t m)
{
	uint8_t h;
	uint16_t minutes;

	cli();
	minutes = ck->t_s / 60;
	h = minutes / 60;
	ck->t_s = 60 * ((uint32_t) h * 60 + m);
	ck->t_s_last = 0;
	sei();
}

uint16_t clock_seconds_to_hhmm_bcd(uint32_t t)
{
	uint16_t minutes;
	uint8_t h, m;

	minutes = t / 60;
	m = minutes % 60;
	h = (minutes / 60) % 24;

	return seven_to_bcd((h * 100) + m);
}

uint16_t clock_encode(volatile struct clock_state *ck)
{
	if (ck->t_s != ck->t_s_last) {
		ck->t_s_last = ck->t_s;
		ck->encoded = clock_seconds_to_hhmm_bcd(ck->t_s);
	}

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
	feeder_transition(&feeder, FEED_TICK);
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
#define MAIN_PROG_PATTERN PAT(E|D|F|G|A, G|A, G|A|C|B, E|F|D|G|C|B|P)
#define MAIN_CANCEL_PATTERN PAT(E|F|A|B, E|D|G|A|C|B, G|A|C, G|A|B|P)

#define MAIN_FEED_TIME 5

enum {
	MAIN_SELM_HOURS,
	MAIN_SELM_MINUTES,
	MAIN_SELM_PROG,
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
	MAIN_SET_PROG,
};

#define PROG_NUM_SLOTS 48

#define PROG_DEFAULT_SLOT_1 ((6 * 60 + 30) / 30)
#define PROG_DEFAULT_SLOT_2 ((12 * 60 + 30) / 30)
#define PROG_DEFAULT_SLOT_3 ((16 * 60 + 30) / 30)
#define PROG_DEFAULT_SLOT_4 ((21 * 60 + 30) / 30)

struct program_state {
	uint8_t slots[PROG_NUM_SLOTS / 8];
};

void program_toggle_slot(struct program_state *p, uint8_t slot)
{
	p->slots[slot >> 3] ^= 1 << (slot & 7);
}

void program_init(struct program_state *p)
{
	uint8_t i;

	for (i = 0; i < PROG_NUM_SLOTS / 8; i ++)
		p->slots[i] = 0;

	program_toggle_slot(p, PROG_DEFAULT_SLOT_1); 
	program_toggle_slot(p, PROG_DEFAULT_SLOT_2); 
	program_toggle_slot(p, PROG_DEFAULT_SLOT_3); 
	program_toggle_slot(p, PROG_DEFAULT_SLOT_4); 
}

uint16_t program_get_slot(struct program_state *p, uint8_t slot, bool *active)
{
	uint32_t t_s;

	t_s = (slot * 86400) / PROG_NUM_SLOTS;
	*active = (p->slots[slot >> 3] & (1 << (slot & 7))) != 0;

	return clock_seconds_to_hhmm_bcd(t_s);
}

#define KR_PERIOD 5

int main(void)
{
	bool last_prs, prs = false;
	bool but_prs;
	uint8_t state = MAIN_BANNER_1;
	uint8_t select_state;
	uint32_t t, t_start = 0;
	uint8_t h, m;
	uint16_t h_bcd, m_bcd, hhmm_bcd;
	uint32_t e;
	uint8_t slot;
	bool active;
	uint8_t kr_counter = 0;
	uint8_t kr_phase = 0;
	bool kr_up = true;
	uint32_t pat;
	
        timer_init();
	pwm_init();
	shaker_init();
        seven_init(&seven);
	feeder_init(&feeder);
        serial_init();
        adc_init();
	clock_init(&clock, 12, 34, 56);
	button_init(&button);
	program_init(&program);
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
				kr_counter ++;
				if (kr_counter == KR_PERIOD) {
					kr_counter = 0;
					if (kr_up) {
						kr_phase ++;
						if (kr_phase == 4) {
							kr_phase = 3;
							kr_up = false;
						}
					} else {
						if (kr_phase)
							kr_phase --;
						else kr_up = true;
					}
				}
				pat = seven_encode(clock_encode(&clock));
				pat |= (uint32_t) SS_P << (kr_phase << 3);
				seven_set_pattern(&seven, pat);
				if (!last_prs && prs)
					state = MAIN_SELM;
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

					case MAIN_SELM_PROG:
						seven_set_pattern(&seven,
							MAIN_PROG_PATTERN);
						if (but_prs)
							state = MAIN_SET_PROG;
						break;

					case MAIN_SELM_FEED:
						seven_set_pattern(&seven,
							MAIN_FEED_PATTERN);
						if (but_prs) {
							feeder_transition(
								&feeder,
								FEED_KITTY);
							state = MAIN_DISP_TIME;
						}
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
				h_bcd = seven_to_bcd(h);
				m_bcd = clock_encode(&clock) & 0x00ff;
				e = seven_encode((h_bcd << 8) | m_bcd);
				if (t & 1) e &= 0x0000ffff;
				seven_set_pattern(&seven, e);
				if (but_prs) {
					clock_set_h(&clock, h);
					state = MAIN_DISP_TIME;
				}
				break;

			case MAIN_SET_M:
				m = adc_get_choice(60);
				m_bcd = seven_to_bcd(m);
				h_bcd = clock_encode(&clock)  >> 8;
				e = seven_encode((h_bcd << 8) | m_bcd);
				if (t & 1) e &= 0xffff0000;
				seven_set_pattern(&seven, e);
				if (but_prs) {
					clock_set_m(&clock, m);
					state = MAIN_DISP_TIME;
				}
				break;

			case MAIN_SET_PROG:
				slot = adc_get_choice(PROG_NUM_SLOTS + 1);

				if (slot == PROG_NUM_SLOTS) {
					seven_set_pattern(&seven,
							MAIN_CANCEL_PATTERN);
					if (but_prs)
						state = MAIN_DISP_TIME;
				} else {
					hhmm_bcd = program_get_slot(&program,
							slot,
							&active);
					e = seven_encode(hhmm_bcd);
					if (active)
						e |= PAT(P,P,P,P);
					seven_set_pattern(&seven, e);

					if (but_prs)
						program_toggle_slot(&program,
								slot);
				}
				break;

			default:
				break;
		}

                seven_process(&seven);
        }
}

/* vim:set ts=8 sw=8 noexpandtab tw=80 cc=80: */
