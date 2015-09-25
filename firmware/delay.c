#include <msp430.h>
#include <stdint.h>
#include "delay.h"

// delay ms
void delay_ms(uint16_t t)
{
    volatile uint16_t i = t;
    while(i--) {
        delay_us(1000);
    }
}

// delay microseconds (4 microsecond resolution)
void delay_us(uint16_t t)
{
    t = (t >> 2);
    volatile uint16_t i = t;
    while(i--) {
        __delay_cycles(CYCLES4US);
    }
}
