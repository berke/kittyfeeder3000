/* seven.c
 *
 * Seven-segment display driver.
 *
 * (C)2011 O. Berke A. Durak
 */

#include <avr/delay.h>

#include "seven.h"
#include "config_seven.h"

/* 0 - normal display
 * 1 to 32 : test mode, show segment i */

#define A SS_A
#define B SS_B
#define C SS_C
#define P SS_P
#define D SS_D
#define E SS_E
#define F SS_F
#define G SS_G

static const uint8_t seven_table[] =
{
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

static inline void seven_delay(void)
{
        asm volatile( "nop" );
}

uint16_t seven_to_bcd(uint16_t x)
{
        uint16_t r = 0;

        if(x >= 10000) return 0x9999;
        while(x >= 1000) { r += 0x1000; x -=   1000; }
        while(x >=  100) { r += 0x0100; x -=    100; }
        while(x >=   10) { r += 0x0010; x -=     10; }
        r += x;
        return r;
}

__attribute__((unused))
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

static void seven_multi_delay(uint8_t c)
{
        _delay_us(500);
}

void seven_init(volatile struct seven_state *q)
{
        SEVEN_DGT_DDR |= SEVEN_DGT_0 | SEVEN_DGT_1 | SEVEN_DGT_2 | SEVEN_DGT_3;
        SEVEN_SHR_DDR |= _BV(SEVEN_SHR_DAT) | _BV(SEVEN_SHR_CLK) |
                _BV(SEVEN_SHR_RST);

        q->word    = 0x1234;
}

uint32_t seven_encode(uint16_t x)
{
        return
                ((uint32_t) seven_table[x & 15]) |
                (((uint32_t) seven_table[(x >> 4) & 15]) << 8) |
                (((uint32_t) seven_table[(x >> 8) & 15]) << 16) |
                (((uint32_t) seven_table[(x >> 12) & 15]) << 24);
}

void seven_set_pattern(volatile struct seven_state *q, uint32_t pattern)
{
        q->pattern[3] = (pattern >> 24) & 255;
        q->pattern[2] = (pattern >> 16) & 255;
        q->pattern[1] = (pattern >>  8) & 255;
        q->pattern[0] = pattern & 255;
}

void seven_set(volatile struct seven_state *q, uint16_t x)
{
        seven_set_pattern(q, seven_encode(x));
}

uint16_t seven_get(volatile struct seven_state *q)
{
        return q->word;
}

void seven_process(volatile struct seven_state *q)
{
        uint8_t p0, p1, p2, p3;
        uint8_t i;

        p0 = q->pattern[0];
        p1 = q->pattern[1];
        p2 = q->pattern[2];
        p3 = q->pattern[3];

        SEVEN_DGT_PORT |= SEVEN_DGT_0 | SEVEN_DGT_1 | SEVEN_DGT_2 | SEVEN_DGT_3;

        SEVEN_SHR_PORT &= ~(_BV(SEVEN_SHR_RST) | _BV(SEVEN_SHR_CLK));
        seven_delay();
        SEVEN_SHR_PORT |= _BV(SEVEN_SHR_RST);
        seven_delay();

        SEVEN_SHR_PORT |= _BV(SEVEN_SHR_DAT);
        seven_delay();
        SEVEN_SHR_PORT |= _BV(SEVEN_SHR_CLK);
        seven_delay();

        SEVEN_SHR_PORT &= ~_BV(SEVEN_SHR_DAT);
        SEVEN_SHR_PORT &= ~_BV(SEVEN_SHR_CLK);

        for(i = 0; i < 8; i ++)
        {
                if(p3 & 1) SEVEN_DGT_PORT &= ~SEVEN_DGT_3;
                seven_multi_delay(1);
                SEVEN_DGT_PORT |= SEVEN_DGT_3;

                if(p2 & 1) SEVEN_DGT_PORT &= ~SEVEN_DGT_2;
                seven_multi_delay(1);
                SEVEN_DGT_PORT |= SEVEN_DGT_2;

                if(p1 & 1) SEVEN_DGT_PORT &= ~SEVEN_DGT_1;
                seven_multi_delay(1);
                SEVEN_DGT_PORT |= SEVEN_DGT_1;

                if(p0 & 1) SEVEN_DGT_PORT &= ~SEVEN_DGT_0;
                seven_multi_delay(1);
                SEVEN_DGT_PORT |= SEVEN_DGT_0;

                p3 >>= 1;
                p2 >>= 1;
                p1 >>= 1;
                p0 >>= 1;

                SEVEN_SHR_PORT |= _BV(SEVEN_SHR_CLK);
                seven_delay();
                SEVEN_SHR_PORT &= ~_BV(SEVEN_SHR_CLK);
        }
}

