/* KittyFeeder 3000 */

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
#include "config.h"

#define CHECK 1
#if CHECK
#define CK(x,y,z) cli(); if(!(y)) { sei(); alarm(x); z; } else { sei(); }
#else
#define CK(x,y,z)
#endif

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
#if CLOCK_ACCELERATED
	TCCR0 = _BV(CS01) | _BV(CS00);
#else
	TCCR0 = _BV(CS02) | _BV(CS00);
#endif
	TCNT0 = 0;
        TIMSK = _BV(TOIE0);
}

void pwm_init(void)
{
	ICR1 = PWM_PERIOD;
	OCR1A = PWM_OFF;
	TCCR1A = _BV(COM1A1) | _BV(WGM11);
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

struct feeder_state {
	uint8_t state;
	uint16_t timer;
	uint16_t shake_time;
};

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
	q->shake_time = FEED_SHAKE_TIME;
	q->timer = 0;
}

void feeder_set_qty(struct feeder_state *q, uint16_t qty)
{
	cli();
	q->shake_time = qty;
	sei();
}

uint8_t feeder_get_state(struct feeder_state *q)
{
	return q->state;
}

uint8_t feeder_transition(struct feeder_state *q, enum feed_event e)
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
			if (q->timer >= q->shake_time) {
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

	return q->state;
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
        ADMUX = _BV(REFS0) | _BV(MUX0) | _BV(MUX2);
        ADCSRA = _BV(ADEN);
}

uint16_t adc_get(void)
{
        ADCSRA |= _BV(ADSC);
        do { } while(ADCSRA & _BV(ADSC));
        return ADC;
}

uint32_t uint32_t_d(uint32_t x, uint32_t y)
{
	return x < y ? y - x : x - y;
}

uint16_t adc_get_choice(uint16_t n)
{
	uint32_t pot;
	static uint32_t pot_last = 0;
	uint32_t middle, middle_last;
	uint32_t i, i_last;

	pot = adc_get();
	i = (pot * n) >> ADC_BITS;
	if (i >= n) i = n - 1;
	middle = (((2 * i + 1) << ADC_BITS) / n) >> 1;

	i_last = (pot_last * n) >> ADC_BITS;
	if (i_last >= n) i_last = n - 1;
	middle_last = (((2 * i_last + 1) << ADC_BITS) / n) >> 1;
	return uint32_t_d(pot, middle_last) < uint32_t_d(pot, middle) ?
		i_last : i;
}

void clock_init(volatile struct clock_state *ck,
		uint8_t hour, uint8_t min, uint8_t sec)
{
	ck->t_s = (uint32_t) sec +
		60UL * ((uint32_t) min + 60UL * (uint32_t) hour);
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

uint32_t clock_get_seconds(volatile struct clock_state *ck)
{
	uint32_t t;

	cli();
	t = ck->t_s;
	sei();
	return t;
}

/* 32µs timer */
ISR(SIG_OVERFLOW0)
{
	clock_tick(&clock);
	feeder_transition(&feeder, FEED_TICK);
}

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

enum {
	MAIN_SELM_HOURS,
	MAIN_SELM_MINUTES,
	MAIN_SELM_PROG,
	MAIN_SELM_QTY,
	MAIN_SELM_FEED,
	MAIN_SELM_CANCEL,
	MAIN_SELM_COUNT
};

enum {
	MAIN_BANNER_1,
	MAIN_BANNER_2,
	MAIN_BANNER_3,
	MAIN_DISP_TIME,
	MAIN_FEED,
	MAIN_SELM,
	MAIN_SET_H,
	MAIN_SET_M,
	MAIN_SET_PROG,
	MAIN_SET_QTY,
};

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

	*active = (p->slots[slot >> 3] & (1 << (slot & 7))) != 0;
	t_s = (uint32_t) slot * PROG_SEC_PER_SLOT;

	return clock_seconds_to_hhmm_bcd(t_s);
}

bool program_check(struct program_state *p, uint32_t t_s)
{
	uint8_t slot = (((uint32_t) t_s) % 86400UL) / PROG_SEC_PER_SLOT;
	bool active;
	uint16_t hhmm;

	hhmm = program_get_slot(p, slot, &active);
	return active;
}

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
	bool p_act = false, p_act_last;
	uint8_t f_s;
	uint8_t kr_counter = 0;
	uint8_t kr_phase = 0;
	bool kr_up = true;
	uint32_t pat;
	uint16_t q;
	
        timer_init();
	pwm_init();
	shaker_init();
        seven_init(&seven);
	feeder_init(&feeder);
        serial_init();
        adc_init();
	clock_init(&clock, 12, 24, 56);
	button_init(&button);
	program_init(&program);
        sei();

        for(;;) {
		t = clock_get_ticks(&clock);
		last_prs = prs;
		prs = button_tick(&button);
		but_prs = !last_prs && prs;

		p_act_last = p_act;
		p_act = program_check(&program, clock_get_seconds(&clock));

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
				pat = seven_encode(clock_encode(&clock));

				if (!p_act_last && p_act) {
					cli();
					f_s = feeder_transition(&feeder,
							FEED_KITTY);
					sei();
					state = MAIN_FEED;
				}

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
				pat |= (uint32_t) SS_P << (kr_phase << 3);
				seven_set_pattern(&seven, pat);
				if (!last_prs && prs)
					state = MAIN_SELM;
				break;

			case MAIN_FEED:
				f_s = feeder_get_state(&feeder);
				if (f_s == FEED_IDLE) {
					state = MAIN_DISP_TIME;
				} else {
					seven_set_pattern(&seven,
							MAIN_FEED_PATTERN);
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

					case MAIN_SELM_PROG:
						seven_set_pattern(&seven,
							MAIN_PROG_PATTERN);
						if (but_prs)
							state = MAIN_SET_PROG;
						break;

					case MAIN_SELM_QTY:
						seven_set_pattern(&seven,
							MAIN_QTY_PATTERN);
						if (but_prs) {
							state = MAIN_SET_QTY;
						}
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

			case MAIN_SET_QTY:
				q = adc_get_choice(FEED_MAX_QTY);
				seven_set(&seven, seven_to_bcd(q));
				if (but_prs) {
					feeder_set_qty(&feeder, q);
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
							slot, &active);
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
