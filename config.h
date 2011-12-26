#ifndef CONFIG_H_20111225
#define CONFIG_H_20111225

#define FAST_DEBUG 0

#define F_CPU 1000000
#define CLOCK_ACCELERATED 1
#define CLOCK_PERIOD_US 262144

#define LED1_DDR DDRC
#define LED1_PORT PORTC
#define LED1_BIT 5

#define PWM_PERIOD 10000
#define PWM_ON 1250
#define PWM_OFF 500

#define KR_PERIOD 5

#if CLOCK_ACCELERATED
#define FEED_BASE_TIME 8
#else
#define FEED_BASE_TIME 1
#endif

#define FEED_OPEN_TIME (5 * FEED_BASE_TIME)
#define FEED_SHAKE_TIME (30 * FEED_BASE_TIME)
#define FEED_SHAKE_MAX_TIME (90 * FEED_BASE_TIME)
#define FEED_PAUSE_TIME (2 * FEED_BASE_TIME)
#define FEED_CLOSE_TIME (5 * FEED_BASE_TIME)
#define FEED_MAX_QTY FEED_SHAKE_TIME

#define SHAKER_PORT PORTD
#define SHAKER_DDR DDRD
#define SHAKER_PIN PIND
#define SHAKER_BIT 2

#define ADC_BITS 10

#define PROG_NUM_SLOTS 48
#define PROG_MIN_PER_SLOT (1440 / PROG_NUM_SLOTS)
#define PROG_SEC_PER_SLOT (60 * PROG_MIN_PER_SLOT)

#define PROG_DEFAULT_SLOT_1 ((6 * 60 + 30) / PROG_MIN_PER_SLOT)
#define PROG_DEFAULT_SLOT_2 ((12 * 60 + 30) / PROG_MIN_PER_SLOT)
#define PROG_DEFAULT_SLOT_3 ((16 * 60 + 30) / PROG_MIN_PER_SLOT)
#define PROG_DEFAULT_SLOT_4 ((21 * 60 + 30) / PROG_MIN_PER_SLOT)

#define BUTTON_PORT PORTC
#define BUTTON_DDR DDRC
#define BUTTON_PIN PINC
#define BUTTON_BIT 4
#define BUTTON_LO_THRESHOLD 2
#define BUTTON_HI_THRESHOLD 6
#define BUTTON_LIMIT 8

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
#define MAIN_QTY_PATTERN PAT(E|F|D|G|C, F|G|A|B, F|D|G|C|P, 0)

#endif
