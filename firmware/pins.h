#define LEDOUT P1OUT
#define LEDDIR P1DIR
#define LED1 (1 << 0)

// uart pins
#define UARTOUT P2OUT
#define UARTSEL P2SEL0
#define UARTDIR P2DIR
#define RXD (1 << 6)
#define TXD (1 << 5)

// switches and LEDs
#define SWADDR0_PIN (1 << 0)
#define SWADDR0_OUT P3OUT
#define SWADDR0_DIR P3DIR

#define SWADDR1_PIN (1 << 1)
#define SWADDR1_OUT P3OUT
#define SWADDR1_DIR P3DIR

#define SWADDR2_PIN (1 << 2)
#define SWADDR2_OUT P3OUT
#define SWADDR2_DIR P3DIR

#define SWADDR3_PIN (1 << 3)
#define SWADDR3_OUT P3OUT
#define SWADDR3_DIR P3DIR

#define SWADDR4_PIN (1 << 3)
#define SWADDR4_OUT P1OUT
#define SWADDR4_DIR P1DIR

#define SWADDR5_PIN (1 << 4)
#define SWADDR0_OUT P1OUT
#define SWADDR0_DIR P1DIR

#define SWDEBUG_PIN (1 << 5)
#define SWADDR0_OUT P3OUT
#define SWADDR0_DIR P3DIR

#define SWAUX0_PIN (1 << 2)
#define SWAUX0_OUT PJOUT
#define SWAUX0_DIR PJDIR

#define LED0_PIN (1 << 1)
#define LED0_OUT PJOUT
#define LED0_DIR PJDIR


// cc110L pins
#define CC_SCK_PIN (1 << 2)
#define CC_SCK_OUT P2OUT
#define CC_SCK_DIR P2DIR
#define CC_SCK_SEL P2SEL0

#define CC_MISO_PIN (1 << 7)
#define CC_MISO_OUT P1OUT
#define CC_MISO_DIR P1DIR
#define CC_MISO_SEL P1SEL0

#define CC_MISO_PIN (1 << 6)
#define CC_MISO_OUT P1OUT
#define CC_MISO_DIR P1DIR
#define CC_MISO_SEL P1SEL0

#define CC_CG0_OUT P1OUT
#define CC_CG0_PIN (1 << 2)
#define CC_CG0_DIR P1DIR

#define CC_CS_OUT P3OUT
#define CC_CS_DIR P3DIR
#define CC_CS_PIN (1 << 7)

#define CC_XTAL_OUT PJOUT
#define CC_XTAL_DIR PJDIR
#define CC_XTAL_PIN (1 << 0)

// tc77 pins
#define TC_OUT P2OUT
#define TC_DIR P2DIR
#define TC_SIO_PIN (1 << 4)
#define TC_CS_PIN (1 << 3)
#define TC_SCK_PIN (1 << 7)

// lsm6ds3 pins
#define LSM_INT1_OUT PJOUT
#define LSM_INT1_DIR PJDIR
#define LSM_INT1_SEL PJSEL0
#define LSM_INT1_PIN (1 << 3)

#define LSM_MISO 
#define LSM_MOSI_OUT P2OUT
#define LSM_MOSI_DIR P2DIR
#define LSM_MOSI_SEL P2SEL0
#define LSM_MOSI_PIN (1 << 0)

#define LSM_MISO_OUT P2OUT
#define LSM_MISO_DIR P2DIR
#define LSM_MISO_SEL P2SEL0
#define LSM_MISO_PIN (1 << 1)

#define LSM_CS_OUT P3OUT
#define LSM_CS_DIR P3DIR
#define LSM_CS_SEL P3SEL0
#define LSM_CS_PIN (1 << 4)

#define LSM_SCK_OUT P1OUT
#define LSM_SCK_DIR P1DIR
#define LSM_SCK_SEL P1SEL0
#define LSM_SCK_PIN (1 << 5)

