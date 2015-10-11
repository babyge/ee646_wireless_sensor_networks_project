#include <msp430.h>
#include <stdint.h>
#include "pins.h"
#include "uart.h"
#include "delay.h"

void init_uart(void)
{
    UCA1CTL1 = UCSWRST; // set UCSWRST (recommended before initializing USCI)
    UARTSEL |= RXD | TXD; // set UART pins to UART mode

    UCA1CTL1 |= UCSSEL1; // SMCLK clock select
    UCA1CTL0 = 0; // 8 bit, 1 stop, no parity, async

    // set baud rate SMCLK / (UCA1BR0 + 256 * UCA1BR1)
    UCA1BR1 = 6;
    UCA1BR0 = 130; // 9600 baud, no oversampling

    UCA1MCTL = 0;
    UCA1CTL1 &= ~(UCSWRST); //clear reset
}

uint8_t uart_put(uint8_t c) {
    uint8_t t;
    uint8_t res = BUFF_NOMINAL;

    dint();

    if(txempty) {
        UCA1TXBUF = c;
        txempty = 0;
    }

    else {
        t = (txiin + 1) % TXBUFFSIZE;
        if(t != txiout) {
            tx_buff[txiin] = c;
            txiin = t;
        }
        else {
            res = BUFF_FULL;
        }

    }

    eint();

    return res;
}

uint8_t uart_grab(void) {
    uint8_t c;

    if(rxempty) {
        return 0;
    }

    else {
        dint();
        c = rx_buff[rxiout];
        rxiout = (rxiout + 1) % RXBUFFSIZE;
        if(rxiout == rxiin) {
            rxempty = 1;
        }
        eint();
    }
    return c;
}

uint16_t uart_grab16(void)
{
    return uart_grab() | (uart_grab() << 8);
}

void uart_put16(uint16_t w)
{
    uart_put(w & 0xFF);
    uart_put((w >> 8) & 0xFF);
}
