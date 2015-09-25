/* jtklein@alaska.edu */

#include <msp430.h>
#include <stdint.h>

#include "spi.h"
#include "pins.h"
#include "delay.h"

void init_clock(void)
{
    ;
}

void init_watchdog(void)
{
    // stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;               
}

void init_gpio(void)
{
    // disable the GPIO power-on default high-impedance mode
    PM5CTL0 &= ~LOCKLPM5;                   
}

int main(void)
{
    init_watchdog();
    init_gpio();
    init_clock();

    LEDDIR |= LED1;                          // Set P1.0 to output direction

    for(;;) {
        LEDOUT ^= LED1;                      // Toggle P1.0 using exclusive-OR
        delay_ms(1000);
    }

    return 0;
}
