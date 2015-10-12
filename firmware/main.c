/* jtklein@alaska.edu */

#include <msp430.h>
#include <stdint.h>
#include <signal.h>

#include "spi.h"
#include "pins.h"
#include "delay.h"
#include "include.h"


extern char paTable[];
extern char paTableLen;

char txBuffer[4];
char rxBuffer[4];
unsigned int i;


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

    LEDDIR |= LED1;
    LEDOUT &= ~LED1;

    CC_XTAL_DIR |= CC_XTAL_PIN;
    CC_XTAL_OUT |= CC_XTAL_PIN;
    
    SWAUX0_DIR &= ~(SWAUX0_PIN);
    SWAUX0_OUT |= SWAUX0_PIN;
    SWAUX0_REN |= SWAUX0_PIN;
    
    SWDEBUG_DIR &= ~(SWDEBUG_PIN);
    SWDEBUG_OUT |= SWDEBUG_PIN;
    SWDEBUG_REN |= SWDEBUG_PIN;


}

void init_cc(void)
{

    // copied from TI's main.c in appnote slaa325a
    // 5ms delay to compensate for time to startup between MSP430 and CC1100/2500
    __delay_cycles(5000);

    TI_CC_SPISetup();                         // Initialize SPI port

    TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx

    writeRFSettings();                        // Write RF settings to config reg
    TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE

    // Configure ports -- switch inputs, LEDs, GDO0 to RX packet info from CCxxxx
    //  TI_CC_SW_PxIES = TI_CC_SW1+TI_CC_SW2+TI_CC_SW3+TI_CC_SW4;//Int on falling edge
    //  TI_CC_SW_PxIFG &= ~(TI_CC_SW1+TI_CC_SW2+TI_CC_SW3+TI_CC_SW4);//Clr flags
    //  TI_CC_SW_PxIE = TI_CC_SW1+TI_CC_SW2+TI_CC_SW3+TI_CC_SW4;//Activate enables
       //TI_CC_SW_PxIES = TI_CC_SW1+TI_CC_SW2;//Int on falling edge
       //TI_CC_SW_PxIFG &= ~(TI_CC_SW1+TI_CC_SW2);//Clr flags
       //TI_CC_SW_PxIE = TI_CC_SW1+TI_CC_SW2;//Activate enables
    //TI_CC_SW_PxREN = TI_CC_SW1;               // Enable Pull-Up Resistor
    //TI_CC_SW_PxOUT = TI_CC_SW1;               // Enable Pull-Up Resistor
    //  TI_CC_LED_PxDIR = TI_CC_LED1 + TI_CC_LED2 + TI_CC_LED3 + TI_CC_LED4; //Outputs
    //TI_CC_LED_PxOUT &= ~(TI_CC_LED1 + TI_CC_LED2); //Outputs
    //TI_CC_LED_PxDIR = TI_CC_LED1 + TI_CC_LED2; //Outputs

 //   TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;       // Int on falling edge (end of pkt)
//    TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear flag
//    TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;        // Enable int on end of packet

    TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
                                            // When a pkt is received, it will
                                            // signal on GDO0 and wake CPU
                                            //
    //__bis_SR_register(GIE);


    return;
}

// The ISR assumes the interrupt came from a press of one of the four buttons
// and therefore does not check the other four inputs.
//#pragma vector=PORT3_VECTOR
//__interrupt void port3_ISR (void)
/*
void __attribute__((interrupt(PORT3_VECTOR))) port3_isr(void)
{
  if(P3IFG & (TI_CC_SW1 + TI_CC_SW2))
  {
    // Build packet
    txBuffer[0] = 2;                           // Packet length
    txBuffer[1] = 0x01;                        // Packet address
    txBuffer[2] = (~TI_CC_SW_PxIN) & 0x0F;     // Load four switch inputs

    RFSendPacket(txBuffer, 3);                 // Send value over RF
  }
  else if(P3IFG & TI_CC_GDO0_PIN)
  {
    char len=2;                               // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
    if (RFReceivePacket(rxBuffer,&len))       // Fetch packet from CCxxxx
      TI_CC_LED_PxOUT ^= rxBuffer[1];         // Toggle LEDs according to pkt data
  }
//  P1IFG &= ~(TI_CC_SW1+TI_CC_SW2+TI_CC_SW3+TI_CC_SW4);//Clr flag that caused int

  TI_CC_SW_PxIFG &= ~(TI_CC_SW1+TI_CC_SW2); // Clr flag that caused int
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // After pkt TX, this flag is set.
}                                           // Clear it.
*/

// The ISR assumes the int came from the pin attached to GDO0 and therefore
// does not check the other seven inputs.  Interprets this as a signal from
// CCxxxx indicating packet received.
//#pragma vector=PORT2_VECTOR
//$__interrupt void port2_ISR (void)
/*
void __attribute__((interrupt(PORT2_VECTOR))) port2_isr(void)
{
  char len=2;                               // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
  if (RFReceivePacket(rxBuffer,&len))       // Fetch packet from CCxxxx
    TI_CC_LED_PxOUT |= LED1; //rxBuffer[1];         // Toggle LEDs according to pkt data

  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;                 // Clear flag
}
*/
int main(void)
{
    init_watchdog();
    init_clock();
    init_gpio();
    init_cc();

    char len = 2;

    for(;;) {
        if(SWDEBUG_IN & SWDEBUG_PIN) {
            if (RFReceivePacket(rxBuffer,&len)) {
               if (rxBuffer[1]) {
                LEDOUT |= LED1;
               }
               else {
                LEDOUT &= ~(LED1);
               }
            }
        }
        else {

            txBuffer[0] = 2;                           // Packet length
            txBuffer[1] = 0x01;                        // Packet address

            if(SWAUX0_IN & SWAUX0_PIN) {
                txBuffer[2] = 1;
                LEDOUT |= LED1;
            }
            else {
                LEDOUT &= ~(LED1);
                txBuffer[2] = 0;
            }

            RFSendPacket(txBuffer, 3);
            delay_ms(200);
        }
    }
          

    return 0;
}


