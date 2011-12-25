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

void seven_init(volatile struct seven_state *q)
{
        SEVEN_DGT_DDR |= SEVEN_DGT_0 | SEVEN_DGT_1 | SEVEN_DGT_2 | SEVEN_DGT_3;
        SEVEN_SHR_DDR |= _BV(SEVEN_SHR_DAT) | _BV(SEVEN_SHR_CLK) |
                _BV(SEVEN_SHR_RST);

        q->word    = 0x1234;
}

void seven_set(volatile struct seven_state *q, uint16_t x)
{
        q->word = x;
}

uint16_t seven_get(volatile struct seven_state *q)
{
        return q->word;
}

void seven_process(volatile struct seven_state *q)
{
        uint16_t w;
        uint8_t p0, p1, p2, p3;
        uint8_t i;

        w = q->word;

        p0 = seven_table[w & 15]; w >>= 4;
        p1 = seven_table[w & 15]; w >>= 4;
        p2 = seven_table[w & 15]; w >>= 4;
        p3 = seven_table[w & 15];

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
                multi_delay(1);
                SEVEN_DGT_PORT |= SEVEN_DGT_3;

                if(p2 & 1) SEVEN_DGT_PORT &= ~SEVEN_DGT_2;
                multi_delay(1);
                SEVEN_DGT_PORT |= SEVEN_DGT_2;

                if(p1 & 1) SEVEN_DGT_PORT &= ~SEVEN_DGT_1;
                multi_delay(1);
                SEVEN_DGT_PORT |= SEVEN_DGT_1;

                if(p0 & 1) SEVEN_DGT_PORT &= ~SEVEN_DGT_0;
                multi_delay(1);
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

